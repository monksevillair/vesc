//
// Created by abuchegger on 06.07.18.
//
#ifndef VESC_ACKERMANN_VEHICLE_H
#define VESC_ACKERMANN_VEHICLE_H

#include <ackermann_msgs/AckermannDrive.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <vesc_ackermann/types.h>
#include <vesc_ackermann/VehicleConfig.h>

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
  Vehicle(const ros::NodeHandle& nh, const MotorFactoryPtr& motor_factory);

  void setVelocity(const ackermann_msgs::AckermannDrive& velocity, const ros::Time& time);
  void setVelocity(const geometry_msgs::Twist& velocity, const ros::Time& time);
  geometry_msgs::Twist getVelocity(const ros::Time& time);
  sensor_msgs::JointState getJointStates(const ros::Time& time);

  boost::optional<double> getSupplyVoltage();

protected:
  void reconfigure(VehicleConfig& config);
  static double limit(double value, double max);

  ros::NodeHandle nh_;
  MotorFactoryPtr motor_factory_;
  VehicleConfig config_;
  dynamic_reconfigure::Server<VehicleConfig> reconfigure_server_;
  std::vector<AxlePtr> axles_;

  double wheelbase_ = 0.0;
  double max_steering_angle_ = 0.0;
};
}

#endif //VESC_ACKERMANN_VEHICLE_H
