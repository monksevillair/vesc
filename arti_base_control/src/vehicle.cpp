//
// Created by abuchegger on 06.07.18.
//
#include <arti_base_control/vehicle.h>
#include <arti_base_control/axle.h>
#include <arti_base_control/utils.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include <functional>
#include <memory>

namespace arti_base_control
{
VehicleVelocityConstraint::VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_)
  : a_v_x(a_v_x_), a_v_y(a_v_y_), a_v_theta(a_v_theta_), b(b_)
{
}

Vehicle::Vehicle(const ros::NodeHandle& nh, const MotorFactoryPtr& motor_factory)
  : nh_(nh), motor_factory_(motor_factory), reconfigure_server_(nh)
{
  reconfigure_server_.setCallback(std::bind(&Vehicle::reconfigure, this, std::placeholders::_1));
}

void Vehicle::reconfigure(VehicleConfig& config)
{
  config_ = config;

  if (config.max_velocity_linear == 0.0)
  {
    ROS_ERROR("Parameter max_velocity_linear is not set");
  }

  if (config.allowed_brake_velocity == 0.0)
  {
    ROS_WARN("Parameter allowed_brake_velocity is not set");
  }

  if (config.brake_velocity == 0.0)
  {
    ROS_WARN("Parameter brake_velocity is not set");
  }

  if (config.brake_current == 0.0)
  {
    ROS_WARN("Parameter brake_current is not set");
  }

  if (axles_.empty())
  {
    XmlRpc::XmlRpcValue axles_param;
    if (nh_.getParam("axles", axles_param))
    {
      if (axles_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        const ros::NodeHandle axles_nh(nh_, "axles");
        for (const XmlRpc::XmlRpcValue::ValueStruct::value_type& axle_param : axles_param)
        {
          axles_.emplace_back(
            std::make_shared<Axle>(ros::NodeHandle(axles_nh, axle_param.first), config_, motor_factory_));
        }
      }
      else
      {
        ROS_ERROR_STREAM("axles parameter has invalid type, must be map");
      }
    }
    else
    {
      ROS_ERROR_STREAM("axles parameter is missing");
    }
  }

  wheelbase_ = config_.wheelbase;
  if (wheelbase_ == 0.0)
  {
    for (const AxlePtr& axle : axles_)
    {
      const AxleConfig& axle_config = axle->getConfig();

      if (axle_config.is_steered)
      {
        wheelbase_ = std::fabs(axle_config.position_x - axle_config.steering_icr_x);
        if (wheelbase_ != 0.0)
        {
          break;
        }
      }
    }

    if (wheelbase_ == 0.0)
    {
      ROS_WARN_STREAM("Wheelbase is not set and could not be determined automatically, this prevents control via"
                        " Ackermann messages");
    }
  }

  max_steering_angle_ = M_PI_2;
  for (const AxlePtr& axle : axles_)
  {
    axle->setVehicleConfig(config_);

    const AxleConfig& axle_config = axle->getConfig();

    if (axle_config.is_steered)
    {
      const double max_steering_angle = std::fabs(normalizeSteeringAngle(std::atan2(
        std::sin(axle_config.steering_angle_max) * wheelbase_,
        std::cos(axle_config.steering_angle_max) * (axle_config.position_x - axle_config.steering_icr_x))));

      max_steering_angle_ = std::min(max_steering_angle_, max_steering_angle);
    }
  }
}

void Vehicle::setVelocity(const ackermann_msgs::AckermannDrive& velocity, const ros::Time& time)
{
  const double steering_angle = limit(normalizeSteeringAngle(velocity.steering_angle), max_steering_angle_);
  const double sin_steering_angle = std::sin(steering_angle);
  const double cos_steering_angle = std::cos(steering_angle);

  // Limit linear velocity to stay below angular velocity limit
  // (angular_velocity = tan(steering_angle) * linear_velocity / wheelbase; this can become infinite when
  // steering_angle approaches 90 degrees):
  double angular_velocity = 0.0;
  double linear_velocity = limit(velocity.speed, config_.max_velocity_linear);
  if (linear_velocity != 0.0 && wheelbase_ != 0.0)
  {
    const double a = std::fabs(linear_velocity * sin_steering_angle);
    const double b = std::fabs(config_.max_velocity_angular * wheelbase_ * cos_steering_angle);
    if (a > b)
    {
      linear_velocity *= b / a;
      angular_velocity = config_.max_velocity_angular * (linear_velocity < 0.0 ? -1.0 : 1.0)
        * (steering_angle < 0.0 ? -1.0 : 1.0);
    }
    else
    {
      angular_velocity = linear_velocity * sin_steering_angle / (wheelbase_ * cos_steering_angle);
    }
  }

  for (const AxlePtr& axle : axles_)
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
  const double linear_velocity = limit(velocity.linear.x, config_.max_velocity_linear);
  double angular_velocity = limit(velocity.angular.z, config_.max_velocity_angular);

  if (angular_velocity != 0)
  {
    const double a = std::fabs(std::cos(max_steering_angle_) * wheelbase_ * angular_velocity);
    const double b = std::fabs(std::sin(max_steering_angle_) * linear_velocity);
    if (a > b)
    {
      angular_velocity *= b / a;
    }
  }

  for (const AxlePtr& axle : axles_)
  {
    const AxleConfig& axle_config = axle->getConfig();

    double axle_steering_angle = 0.0;
    if (axle_config.is_steered)
    {
      axle_steering_angle = normalizeSteeringAngle(
        std::atan2((axle_config.position_x - axle_config.steering_icr_x) * angular_velocity, linear_velocity));
    }

    axle->setVelocity(linear_velocity, angular_velocity, axle_steering_angle, time);
  }
}

geometry_msgs::Twist Vehicle::getVelocity(const ros::Time& time)
{
  // Compute vehicle velocity using pseudo inverse of velocity constraints:
  VehicleVelocityConstraints constraints;
  for (const AxlePtr& axle : axles_)
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
  for (const AxlePtr& axle : axles_)
  {
    axle->getJointStates(time, joint_states);
  }
  return joint_states;
}

boost::optional<double> Vehicle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  for (const AxlePtr& axle : axles_)
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

double Vehicle::limit(const double value, const double max)
{
  return std::min(std::max(-max, value), max);
}
}
