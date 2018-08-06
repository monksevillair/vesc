//
// Created by abuchegger on 06.08.18.
//
#ifndef VESC_DRIVER_VESC_DRIVER_NODE_H
#define VESC_DRIVER_VESC_DRIVER_NODE_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <vesc_driver/types.h>

namespace vesc_driver
{
class VescDriverNode
{
public:
  VescDriverNode() = default;
  void start();

protected:
  void processMotorControllerState(const MotorControllerState& motor_controller_state);

  void processDutyCycleCommand(const std_msgs::Float64ConstPtr& duty_cycle_command);
  void processCurrentCommand(const std_msgs::Float64ConstPtr& current_command);
  void processBrakeCommand(const std_msgs::Float64ConstPtr& brake_command);
  void processSpeedCommand(const std_msgs::Float64ConstPtr& speed_command);
  void processPositionCommand(const std_msgs::Float64ConstPtr& position_command);

  bool serveFirmwareVersion(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);

  static void publishData(ros::Publisher& publisher, int32_t data);
  static void publishData(ros::Publisher& publisher, double data);

  ros::NodeHandle nh_{"~"};

  ros::Subscriber duty_cycle_subscriber_;
  ros::Subscriber current_subscriber_;
  ros::Subscriber brake_subscriber_;
  ros::Subscriber speed_subscriber_;
  ros::Subscriber position_subscriber_;

  ros::ServiceServer get_firmware_version_server_;

  ros::Publisher fault_code_publisher_;
  ros::Publisher voltage_input_publisher_;
  ros::Publisher current_motor_publisher_;
  ros::Publisher current_input_publisher_;
  ros::Publisher speed_publisher_;
  ros::Publisher position_publisher_;
  ros::Publisher duty_cycle_publisher_;
  ros::Publisher charge_drawn_publisher_;
  ros::Publisher charge_regen_publisher_;
  ros::Publisher displacement_publisher_;
  ros::Publisher distance_traveled_publisher_;

  VescDriverInterfacePtr driver_;
};
}

#endif //VESC_DRIVER_VESC_DRIVER_NODE_H
