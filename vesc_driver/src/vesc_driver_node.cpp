//
// Created by abuchegger on 06.08.18.
//

#include <vesc_driver/vesc_driver_node.h>
#include <ros/init.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/packet.h>
#include <vesc_driver/serial_transport.h>
#include <vesc_driver/vesc_driver_impl.h>
#include <vesc_driver/vesc_driver_interface.h>

namespace vesc_driver
{
void VescDriverNode::start()
{
  fault_code_publisher_ = nh_.advertise<std_msgs::Int32>("state/fault_code", 1);
  voltage_input_publisher_ = nh_.advertise<std_msgs::Float64>("state/voltage_input", 1);
  current_motor_publisher_ = nh_.advertise<std_msgs::Float64>("state/current_motor", 1);
  current_input_publisher_ = nh_.advertise<std_msgs::Float64>("state/current_input", 1);
  speed_publisher_ = nh_.advertise<std_msgs::Float64>("state/speed", 1);
  position_publisher_ = nh_.advertise<std_msgs::Float64>("state/position", 1);
  duty_cycle_publisher_ = nh_.advertise<std_msgs::Float64>("state/duty_cycle", 1);
  charge_drawn_publisher_ = nh_.advertise<std_msgs::Float64>("state/charge_drawn", 1);
  charge_regen_publisher_ = nh_.advertise<std_msgs::Float64>("state/charge_regen", 1);
  displacement_publisher_ = nh_.advertise<std_msgs::Int32>("state/displacement", 1);
  distance_traveled_publisher_ = nh_.advertise<std_msgs::Int32>("state/distance_traveled", 1);

  const SerialTransportPtr transport = std::make_shared<SerialTransport>(
    static_cast<uint8_t>(nh_.param<int>("transport_controller_id", -1)), nh_.param<std::string>("port", ""));
  transport->connect();

  driver_ = std::make_shared<VescDriverImpl>(
    std::chrono::duration<double>(nh_.param<double>("sleep_duration", 0.1)),
    std::bind(&VescDriverNode::processMotorControllerState, this, std::placeholders::_1),
    static_cast<uint8_t>(nh_.param<int>("controller_id", -1)), transport);

  duty_cycle_subscriber_ = nh_.subscribe("command/duty_cycle", 1, &VescDriverNode::processDutyCycleCommand, this);
  current_subscriber_ = nh_.subscribe("command/current", 1, &VescDriverNode::processCurrentCommand, this);
  brake_subscriber_ = nh_.subscribe("command/brake", 1, &VescDriverNode::processBrakeCommand, this);
  speed_subscriber_ = nh_.subscribe("command/speed", 1, &VescDriverNode::processSpeedCommand, this);
  position_subscriber_ = nh_.subscribe("command/position", 1, &VescDriverNode::processPositionCommand, this);

  get_firmware_version_server_ = nh_.advertiseService(
    "command/get_firmware_version", &VescDriverNode::serveFirmwareVersion, this);
}

void VescDriverNode::processMotorControllerState(const MotorControllerState& motor_controller_state)
{
  publishData(fault_code_publisher_, static_cast<int32_t>(motor_controller_state.fault_code));
  publishData(voltage_input_publisher_, motor_controller_state.voltage_input);
  publishData(current_motor_publisher_, motor_controller_state.current_motor);
  publishData(current_input_publisher_, motor_controller_state.current_input);
  publishData(speed_publisher_, motor_controller_state.speed);
  publishData(position_publisher_, motor_controller_state.position);
  publishData(duty_cycle_publisher_, motor_controller_state.duty_cycle);
  publishData(charge_drawn_publisher_, motor_controller_state.charge_drawn);
  publishData(charge_regen_publisher_, motor_controller_state.charge_regen);
  publishData(displacement_publisher_, motor_controller_state.displacement);
  publishData(distance_traveled_publisher_, motor_controller_state.distance_traveled);
}

void VescDriverNode::processDutyCycleCommand(const std_msgs::Float64ConstPtr& duty_cycle_command)
{
  driver_->setDutyCycle(duty_cycle_command->data);
}

void VescDriverNode::processCurrentCommand(const std_msgs::Float64ConstPtr& current_command)
{
  driver_->setCurrent(current_command->data);
}

void VescDriverNode::processBrakeCommand(const std_msgs::Float64ConstPtr& brake_command)
{
  driver_->setBrake(brake_command->data);
}

void VescDriverNode::processSpeedCommand(const std_msgs::Float64ConstPtr& speed_command)
{
  driver_->setSpeed(speed_command->data);
}

void VescDriverNode::processPositionCommand(const std_msgs::Float64ConstPtr& position_command)
{
  driver_->setPosition(position_command->data);
}

bool VescDriverNode::serveFirmwareVersion(std_srvs::TriggerRequest& /*request*/, std_srvs::TriggerResponse& response)
{
  const FirmwareVersion firmware_version = driver_->getFirmwareVersion();
  response.success = 1;
  response.message = std::to_string(firmware_version.major_version) + "."
    + std::to_string(firmware_version.minor_version);
  return true;
}

void VescDriverNode::publishData(ros::Publisher& publisher, int32_t data)
{
  std_msgs::Int32 msg;
  msg.data = data;
  publisher.publish(msg);
}

void VescDriverNode::publishData(ros::Publisher& publisher, double data)
{
  std_msgs::Float64 msg;
  msg.data = data;
  publisher.publish(msg);
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_driver");
  vesc_driver::VescDriverNode node;
  node.start();
  ros::spin();
  return 0;
}
