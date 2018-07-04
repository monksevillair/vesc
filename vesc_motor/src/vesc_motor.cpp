//
// Created by abuchegger on 04.07.18.
//
#include <vesc_motor/vesc_motor.h>
#include <limits>
#include <vesc_driver/vesc_driver_impl.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_motor
{
VescMotor::VescMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory,
                     double execution_duration)
  : private_nh_(private_nh), transport_factory_(transport_factory), execution_duration_(execution_duration),
    state_handler_function_(std::bind(&VescMotor::processMotorControllerState, this, std::placeholders::_1)),
    supply_voltage_(std::numeric_limits<double>::quiet_NaN())
{
}

double VescMotor::getSupplyVoltage()
{
  return supply_voltage_;
}

void VescMotor::updateDriver(bool use_mockup)
{
  ROS_DEBUG_STREAM("VescMotor::updateDriver::1");

  if (use_mockup)
  {
    ROS_DEBUG_STREAM("VescMotor::updateDriver::3");

    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_))
    {
      ROS_DEBUG_STREAM("VescMotor::updateDriver::4");

      driver_.reset(new vesc_driver::VescDriverMockup(execution_duration_, state_handler_function_));
    }
  }
  else
  {
    ROS_DEBUG_STREAM("VescMotor::updateDriver::5");

    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverImpl>(driver_))
    {
      ROS_DEBUG_STREAM("VescMotor::updateDriver::6");

      int controller_id;
      private_nh_.getParam("controller_id", controller_id);

      ROS_DEBUG_STREAM("VescMotor::updateDriver::7");
      ROS_DEBUG_STREAM("VescMotor::updateDriver::7.1 controller_id: " << controller_id);

      std::shared_ptr<vesc_driver::Transport> transport;

      if (private_nh_.hasParam("port"))
      {
        ROS_DEBUG_STREAM("VescMotor::updateDriver::8");

        std::string port;
        private_nh_.getParam("port", port);

        ROS_DEBUG_STREAM("VescMotor::updateDriver::8.1 port: " << port);

        transport = transport_factory_->createSerialTransport(controller_id, port);
      }
      else
      {
        ROS_DEBUG_STREAM("VescMotor::updateDriver::9");

        std::string transport_name;
        private_nh_.getParam("transport_name", transport_name);

        transport = transport_factory_->getTransport(transport_name);
      }

      driver_.reset(new vesc_driver::VescDriverImpl(execution_duration_, state_handler_function_, controller_id,
                                                    transport));

      ROS_DEBUG_STREAM("VescMotor::updateDriver::10");
    }
  }
}

void VescMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  supply_voltage_ = state.voltage_input;
}
}
