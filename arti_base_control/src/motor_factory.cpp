//
// Created by abuchegger on 10.07.18.
//
#include <arti_base_control/motor_factory.h>
#include <arti_base_control/drive_motor.h>
#include <arti_base_control/steering_motor.h>
#include <vesc_motor/driver_factory.h>
#include <vesc_motor/transport_factory.h>

namespace arti_base_control
{
MotorFactory::MotorFactory(const ros::NodeHandle& nh, double control_interval, bool publish_motor_states,
                           bool use_mockup)
  : control_interval_(control_interval), publish_motor_states_(publish_motor_states)
{
  driver_factory_ = std::make_shared<vesc_motor::DriverFactory>(std::make_shared<vesc_motor::TransportFactory>(nh),
                                                                use_mockup);
}

DriveMotorPtr MotorFactory::createDriveMotor(ros::NodeHandle& private_nh)
{
  const DriveMotorPtr motor = std::make_shared<VescDriveMotor>(private_nh, driver_factory_, control_interval_);
  if (publish_motor_states_)
  {
    return std::make_shared<PublishingDriveMotor>(private_nh, motor);
  }
  return motor;
}

SteeringMotorPtr MotorFactory::createSteeringMotor(ros::NodeHandle& private_nh)
{
  const SteeringMotorPtr motor = std::make_shared<VescSteeringMotor>(private_nh, driver_factory_, control_interval_);
  if (publish_motor_states_)
  {
    return std::make_shared<PublishingSteeringMotor>(private_nh, motor);
  }
  return motor;
}
}
