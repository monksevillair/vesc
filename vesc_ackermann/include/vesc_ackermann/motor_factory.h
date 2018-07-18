//
// Created by abuchegger on 10.07.18.
//
#ifndef VESC_ACKERMANN_MOTOR_FACTORY_H
#define VESC_ACKERMANN_MOTOR_FACTORY_H

#include <ros/node_handle.h>
#include <vesc_ackermann/types.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_ackermann
{
class MotorFactory
{
public:
  MotorFactory(const ros::NodeHandle& nh, double control_interval, bool publish_motor_states);

  DriveMotorPtr createDriveMotor(ros::NodeHandle& private_nh);
  SteeringMotorPtr createSteeringMotor(ros::NodeHandle& private_nh);

protected:
  std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory_;
  double control_interval_;
  bool publish_motor_states_;
};
}

#endif //VESC_ACKERMANN_MOTOR_FACTORY_H
