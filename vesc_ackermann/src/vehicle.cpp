//
// Created by abuchegger on 06.07.18.
//
#include <vesc_ackermann/vehicle.h>
#include <Eigen/Core>
#include <Eigen/QR>

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

  front_axle_.setCommonConfig(common_config_);
  front_axle_.setIcrX(icr_x);
  rear_axle_.setCommonConfig(common_config_);
  rear_axle_.setIcrX(icr_x);
}

void Vehicle::setVelocity(const ackermann_msgs::AckermannDrive& velocity)
{
  // TODO: this assumes that the steering velocity in the message is related to the vehicle wheelbase. Is this correct?
  const double steering_angle = std::max(std::min<double>(velocity.steering_angle, common_config_.max_steering_angle),
                                         -common_config_.max_steering_angle);
  const double normalized_steering_angle = std::atan2(std::tan(steering_angle), common_config_.wheelbase);

  const ros::Time now = ros::Time::now();
  front_axle_.setVelocity(velocity.speed, normalized_steering_angle, now);
  rear_axle_.setVelocity(velocity.speed, normalized_steering_angle, now);
}

void Vehicle::setVelocity(const geometry_msgs::Twist& velocity)
{
  const double normalized_steering_angle = std::atan2(velocity.angular.z, velocity.linear.x);

  const ros::Time now = ros::Time::now();
  front_axle_.setVelocity(velocity.linear.x, normalized_steering_angle, now);
  rear_axle_.setVelocity(velocity.linear.x, normalized_steering_angle, now);
}

geometry_msgs::Twist Vehicle::getVelocity(const ros::Time& time)
{
  // Compute vehicle velocity using pseudo inverse of velocity constraints:
  VehicleVelocityConstraints constraints;
  front_axle_.getVelocityConstraints(time, constraints);
  rear_axle_.getVelocityConstraints(time, constraints);

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
  front_axle_.getJointStates(time, joint_states);
  rear_axle_.getJointStates(time, joint_states);
  return joint_states;
}
}
