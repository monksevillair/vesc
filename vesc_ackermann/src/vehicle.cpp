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

  double icr_x = 0.0;
  if (front_axle_.getConfig().is_steered && rear_axle_.getConfig().is_steered)
  {
    icr_x = front_axle_.getConfig().position_x * common_config_.icr_line_relative_position
      + rear_axle_.getConfig().position_x * (1.0 - common_config_.icr_line_relative_position);
  }
  else if (front_axle_.getConfig().is_steered)
  {
    icr_x = rear_axle_.getConfig().position_x;
  }
  else if (rear_axle_.getConfig().is_steered)
  {
    icr_x = front_axle_.getConfig().position_x;
  }

  wheelbase_ = std::max(std::fabs(front_axle_.getConfig().position_x - icr_x),
                        std::fabs(rear_axle_.getConfig().position_x - icr_x));

  for (Axle* const axle : axles_)
  {
    axle->setCommonConfig(common_config_);
    axle->setIcrX(icr_x);
  }
}

void Vehicle::setVelocity(const ackermann_msgs::AckermannDrive& velocity)
{
  const double linear_velocity = std::max(std::min<double>(velocity.speed, common_config_.max_velocity_linear),
                                          -common_config_.max_velocity_linear);
  const double steering_angle = limitSteeringAngle(velocity.steering_angle);

  const ros::Time now = ros::Time::now();
  for (Axle* const axle : axles_)
  {
    axle->setVelocity(linear_velocity, steering_angle, wheelbase_, now);
  }
}

void Vehicle::setVelocity(const geometry_msgs::Twist& velocity)
{
  const double linear_velocity = std::max(std::min<double>(velocity.linear.x, common_config_.max_velocity_linear),
                                          -common_config_.max_velocity_linear);
  const double steering_angle = limitSteeringAngle(std::atan2(wheelbase_ * velocity.angular.z, velocity.linear.x));

  const ros::Time now = ros::Time::now();
  for (Axle* const axle : axles_)
  {
    axle->setVelocity(linear_velocity, steering_angle, wheelbase_, now);
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
  return std::max(-common_config_.max_steering_angle,
                  std::min(normalizeSteeringAngle(steering_angle), common_config_.max_steering_angle));
}
}
