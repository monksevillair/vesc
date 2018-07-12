//
// Created by abuchegger on 06.07.18.
//
#ifndef VESC_ACKERMANN_VEHICLE_H
#define VESC_ACKERMANN_VEHICLE_H

#include <ackermann_msgs/AckermannDrive.h>
#include <array>
#include <boost/optional.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <vesc_ackermann/axle.h>
#include <vesc_ackermann/types.h>

namespace vesc_ackermann
{
struct VehicleVelocityConstraint
{
  VehicleVelocityConstraint() = default;
  VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_);

  double a_v_x = 0.0;
  double a_v_y = 0.0;
  double a_v_theta = 0.0;
  double b = 0.0;
};

class Vehicle
{
public:
  Vehicle(const ros::NodeHandle& nh, const AckermannConfig& common_config, const MotorFactoryPtr& motor_factory);

  void setCommonConfig(const AckermannConfig& common_config);

  void setVelocity(const ackermann_msgs::AckermannDrive& velocity);
  void setVelocity(const geometry_msgs::Twist& velocity);
  geometry_msgs::Twist getVelocity(const ros::Time& time);
  sensor_msgs::JointState getJointStates(const ros::Time& time);

  boost::optional<double> getSupplyVoltage();

protected:
  AckermannConfig common_config_;
  Axle front_axle_;
  Axle rear_axle_;
  std::array<Axle*, 2> axles_;
  double wheelbase_ = 0.0;
};
}

#endif //VESC_ACKERMANN_VEHICLE_H
