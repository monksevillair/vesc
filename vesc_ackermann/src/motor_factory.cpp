//
// Created by abuchegger on 10.07.18.
//
#include <vesc_ackermann/motor_factory.h>
#include <vesc_ackermann/drive_motor.h>
#include <vesc_ackermann/steering_motor.h>

namespace vesc_ackermann
{
MotorFactory::MotorFactory(const ros::NodeHandle& nh, double control_interval, bool publish_motor_states)
  : transport_factory_(std::make_shared<vesc_motor::VescTransportFactory>(nh)), control_interval_(control_interval),
    publish_motor_states_(publish_motor_states)
{
}

DriveMotorPtr MotorFactory::createDriveMotor(ros::NodeHandle& private_nh)
{
  const DriveMotorPtr motor = std::make_shared<VescDriveMotor>(private_nh, transport_factory_, control_interval_);
  if (publish_motor_states_)
  {
    return std::make_shared<PublishingDriveMotor>(private_nh, motor);
  }
  return motor;
}

SteeringMotorPtr MotorFactory::createSteeringMotor(ros::NodeHandle& private_nh)
{
  const SteeringMotorPtr motor = std::make_shared<VescSteeringMotor>(private_nh, transport_factory_, control_interval_);
  if (publish_motor_states_)
  {
    return std::make_shared<PublishingSteeringMotor>(private_nh, motor);
  }
  return motor;
}
}
