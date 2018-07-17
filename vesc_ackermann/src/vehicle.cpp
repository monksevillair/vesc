//
// Created by abuchegger on 06.07.18.
//
#include <vesc_ackermann/vehicle.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include <vesc_ackermann/utils.h>

namespace vesc_ackermann
{
VehicleVelocityConstraint::VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_)
  : a_v_x(a_v_x_), a_v_y(a_v_y_), a_v_theta(a_v_theta_), b(b_)
{
}

Vehicle::Vehicle(const ros::NodeHandle& nh, const AckermannConfig& common_config,
                 const MotorFactoryPtr& motor_factory)
  : front_axle_(ros::NodeHandle(nh, "front_axle"), common_config, motor_factory),
    rear_axle_(ros::NodeHandle(nh, "rear_axle"), common_config, motor_factory)
{
  axles_.at(0) = &front_axle_;
  axles_.at(1) = &rear_axle_;
  setCommonConfig(common_config);
}

void Vehicle::setCommonConfig(const AckermannConfig& common_config)
{
  common_config_ = common_config;

  const AxleConfig& front_axle_config = front_axle_.getConfig();
  const AxleConfig& rear_axle_config = rear_axle_.getConfig();

  if (front_axle_config.is_steered)
  {
    wheelbase_ = std::fabs(front_axle_config.position_x - front_axle_config.steering_icr_x);
  }
  else if (rear_axle_config.is_steered)
  {
    wheelbase_ = std::fabs(rear_axle_config.position_x - rear_axle_config.steering_icr_x);
  }
  else
  {
    ROS_ERROR_STREAM("Neither front nor rear axle is steered, this doesn't make much sense");
    // Despite this configuration makes little sense, set wheelbase to something sensible:
    wheelbase_ = std::fabs(front_axle_config.position_x - rear_axle_config.position_x);
  }

  max_steering_angle_ = M_PI_2;
  for (Axle* const axle : axles_)
  {
    axle->setCommonConfig(common_config_);

    const AxleConfig& axle_config = axle->getConfig();

    if (axle_config.is_steered && axle_config.steering_angle_max > 0.0)
    {
      const double max_steering_angle = normalizeSteeringAngle(std::atan2(
        std::sin(axle_config.steering_angle_max) * wheelbase_,
        std::cos(axle_config.steering_angle_max) * (axle_config.position_x - axle_config.steering_icr_x)));

      max_steering_angle_ = std::min(max_steering_angle_, max_steering_angle);
    }
  }
}

void Vehicle::setVelocity(const ackermann_msgs::AckermannDrive& velocity, const ros::Time& time)
{
  const double steering_angle = limitSteeringAngle(velocity.steering_angle);
  const double sin_steering_angle = std::sin(steering_angle);
  const double cos_steering_angle = std::cos(steering_angle);

  // Limit linear velocity to stay below angular velocity limit
  // (angular_velocity = tan(steering_angle) * linear_velocity / wheelbase; this can become infinite when
  // steering_angle approaches 90 degrees):
  double angular_velocity = 0.0;
  double linear_velocity = std::min(std::max<double>(-common_config_.max_velocity_linear, velocity.speed),
                                    common_config_.max_velocity_linear);
  if (linear_velocity != 0.0)
  {
    const double a = std::fabs(linear_velocity * sin_steering_angle);
    const double b = std::fabs(common_config_.max_velocity_angular * wheelbase_ * cos_steering_angle);
    if (a > b)
    {
      linear_velocity *= b / a;
      angular_velocity = common_config_.max_velocity_angular * (linear_velocity < 0.0 ? -1.0 : 1.0)
        * (steering_angle < 0.0 ? -1.0 : 1.0);
    }
    else
    {
      angular_velocity = linear_velocity * sin_steering_angle / (wheelbase_ * cos_steering_angle);
    }
  }

  for (Axle* const axle : axles_)
  {
    const AxleConfig& axle_config = axle->getConfig();

    double axle_steering_angle = 0.0;
    if (axle_config.is_steered)
    {
      axle_steering_angle = normalizeSteeringAngle(std::atan2(
        sin_steering_angle * (axle_config.position_x - axle_config.steering_icr_x), cos_steering_angle * wheelbase_));
    }

    axle->setVelocity(linear_velocity, angular_velocity, axle_steering_angle, time);
  }
}

void Vehicle::setVelocity(const geometry_msgs::Twist& velocity, const ros::Time& time)
{
  const double linear_velocity = std::min(std::max(-common_config_.max_velocity_linear, velocity.linear.x),
                                          common_config_.max_velocity_linear);
  double angular_velocity = std::min(std::max(-common_config_.max_velocity_angular, velocity.angular.z),
                                     common_config_.max_velocity_angular);

  if (angular_velocity != 0)
  {
    const double a = std::fabs(std::cos(max_steering_angle_) * wheelbase_ * angular_velocity);
    const double b = std::fabs(std::sin(max_steering_angle_) * linear_velocity);
    if (a > b)
    {
      angular_velocity *= b / a;
    }
  }

  for (Axle* const axle : axles_)
  {
    const AxleConfig& axle_config = axle->getConfig();
    const double axle_steering_angle
      = std::atan2((axle_config.position_x - axle_config.steering_icr_x) * angular_velocity, linear_velocity);

    axle->setVelocity(linear_velocity, angular_velocity, axle_steering_angle, time);
  }
}

geometry_msgs::Twist Vehicle::getVelocity(const ros::Time& time)
{
  // Compute vehicle velocity using pseudo inverse of velocity constraints:
  VehicleVelocityConstraints constraints;
  for (Axle* const axle : axles_)
  {
    axle->getVelocityConstraints(time, constraints);
  }

  Eigen::MatrixXd a(Eigen::MatrixXd::Zero(constraints.size(), 3));
  Eigen::VectorXd b(Eigen::VectorXd::Zero(constraints.size()));
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    a(i, 0) = constraints[i].a_v_x;
    a(i, 1) = constraints[i].a_v_y;
    a(i, 2) = constraints[i].a_v_theta;
    b(i) = constraints[i].b;
  }

  const Eigen::Vector3d x = a.fullPivHouseholderQr().solve(b);

  geometry_msgs::Twist velocity;
  velocity.linear.x = x(0);
  velocity.linear.y = x(1);
  velocity.angular.z = x(2);
  return velocity;
}

sensor_msgs::JointState Vehicle::getJointStates(const ros::Time& time)
{
  sensor_msgs::JointState joint_states;
  joint_states.header.stamp = time;
  for (Axle* const axle : axles_)
  {
    axle->getJointStates(time, joint_states);
  }
  return joint_states;
}

boost::optional<double> Vehicle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  for (Axle* const axle : axles_)
  {
    const boost::optional<double> axle_supply_voltage = axle->getSupplyVoltage();
    if (axle_supply_voltage)
    {
      supply_voltage += *axle_supply_voltage;
      ++num_measurements;
    }
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }

  return boost::none;
}

double Vehicle::limitSteeringAngle(const double steering_angle) const
{
  return std::max(-max_steering_angle_, std::min(normalizeSteeringAngle(steering_angle), max_steering_angle_));
}
}
