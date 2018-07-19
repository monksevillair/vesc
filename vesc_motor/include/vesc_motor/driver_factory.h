//
// Created by abuchegger on 19.07.18.
//
#ifndef VESC_MOTOR_DRIVER_FACTORY_H
#define VESC_MOTOR_DRIVER_FACTORY_H

#include <chrono>
#include <ros/node_handle.h>
#include <vesc_driver/types.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_motor/types.h>

namespace vesc_motor
{
class DriverFactory
{
public:
  DriverFactory(TransportFactoryPtr transport_factory, bool use_mockup);

  vesc_driver::VescDriverInterfacePtr createDriver(
    const ros::NodeHandle& nh, std::chrono::duration<double> execution_duration,
    vesc_driver::VescDriverInterface::StateHandlerFunction state_handler_function);

protected:
  TransportFactoryPtr transport_factory_;
  bool use_mockup_;
};
}

#endif //VESC_MOTOR_DRIVER_FACTORY_H
