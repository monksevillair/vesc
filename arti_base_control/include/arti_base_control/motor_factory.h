//
// Created by abuchegger on 10.07.18.
//
#ifndef ARTI_BASE_CONTROL_MOTOR_FACTORY_H
#define ARTI_BASE_CONTROL_MOTOR_FACTORY_H

#include <arti_base_control/types.h>
#include <ros/node_handle.h>
#include <vesc_motor/types.h>

namespace arti_base_control
{
class MotorFactory
{
public:
  MotorFactory(const ros::NodeHandle& nh, double control_interval, bool publish_motor_states, bool use_mockup);

  DriveMotorPtr createDriveMotor(ros::NodeHandle& private_nh);
  SteeringMotorPtr createSteeringMotor(ros::NodeHandle& private_nh);

protected:
  vesc_motor::DriverFactoryPtr driver_factory_;
  double control_interval_;
  bool publish_motor_states_;
};
}

#endif //ARTI_BASE_CONTROL_MOTOR_FACTORY_H
