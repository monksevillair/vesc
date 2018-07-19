//
// Created by abuchegger on 04.07.18.
//
#include <vesc_motor/vesc_motor.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_motor/driver_factory.h>
#include <limits>

namespace vesc_motor
{
VescMotor::VescMotor(const ros::NodeHandle& private_nh, const DriverFactoryPtr& driver_factory,
                     const std::chrono::duration<double>& execution_duration)
  : private_nh_(private_nh), driver_factory_(driver_factory), execution_duration_(execution_duration),
    supply_voltage_(std::numeric_limits<double>::quiet_NaN())
{
}

double VescMotor::getSupplyVoltage()
{
  return supply_voltage_;
}

void VescMotor::createDriver()
{
  driver_ = driver_factory_->createDriver(
    private_nh_, execution_duration_,
    std::bind(&VescMotor::callProcessMotorControllerState, this, std::placeholders::_1));
}

void VescMotor::callProcessMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  // This indirection must be done because std::bind in the constructor doesn't work right with virtual functions.
  processMotorControllerState(state);
  supply_voltage_ = state.voltage_input;
}
}
