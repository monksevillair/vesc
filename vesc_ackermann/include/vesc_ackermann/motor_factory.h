//
// Created by abuchegger on 10.07.18.
//
#ifndef VESC_ACKERMANN_MOTOR_FACTORY_H
#define VESC_ACKERMANN_MOTOR_FACTORY_H

#include <vesc_motor/vesc_drive_motor.h>
#include <vesc_motor/vesc_steering_motor.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_ackermann
{
class MotorFactory
{
public:
  MotorFactory(const ros::NodeHandle& nh, double control_interval);

  std::shared_ptr<vesc_motor::VescDriveMotor> createDriveMotor(ros::NodeHandle& private_nh);
  std::shared_ptr<vesc_motor::VescSteeringMotor> createSteeringMotor(ros::NodeHandle& private_nh);

protected:
  std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory_;
  double control_interval_;
};
}

#endif //VESC_ACKERMANN_MOTOR_FACTORY_H
