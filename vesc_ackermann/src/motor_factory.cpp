//
// Created by abuchegger on 10.07.18.
//
#include <vesc_ackermann/motor_factory.h>

namespace vesc_ackermann
{
MotorFactory::MotorFactory(const ros::NodeHandle& nh, double control_interval)
  : transport_factory_(std::make_shared<vesc_motor::VescTransportFactory>(nh)), control_interval_(control_interval)
{
}

std::shared_ptr<vesc_motor::VescDriveMotor> MotorFactory::createDriveMotor(ros::NodeHandle& private_nh)
{
  return std::make_shared<vesc_motor::VescDriveMotor>(private_nh, transport_factory_, control_interval_);
}

std::shared_ptr<vesc_motor::VescSteeringMotor> MotorFactory::createSteeringMotor(ros::NodeHandle& private_nh)
{
  return std::make_shared<vesc_motor::VescSteeringMotor>(private_nh, transport_factory_, control_interval_);
}
}
