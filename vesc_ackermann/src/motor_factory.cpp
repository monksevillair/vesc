//
// Created by abuchegger on 10.07.18.
//
#include <vesc_ackermann/motor_factory.h>

namespace vesc_ackermann
{
MotorFactory::MotorFactory(const ros::NodeHandle& nh)
  : transport_factory_(std::make_shared<vesc_motor::VescTransportFactory>(nh))
{
}

MotorFactory::MotorFactory(const ros::NodeHandle& nh, const std::string& parameter_name)
  : transport_factory_(std::make_shared<vesc_motor::VescTransportFactory>(nh, parameter_name))
{
}

std::shared_ptr<vesc_motor::VescDriveMotor> MotorFactory::createDriveMotor(ros::NodeHandle& private_nh,
                                                                           double control_interval)
{
  return std::make_shared<vesc_motor::VescDriveMotor>(private_nh, transport_factory_, control_interval);
}

std::shared_ptr<vesc_motor::VescSteeringMotor> MotorFactory::createSteeringMotor(ros::NodeHandle& private_nh,
                                                                                 double control_interval)
{
  return std::make_shared<vesc_motor::VescSteeringMotor>(private_nh, transport_factory_, control_interval);
}
}
