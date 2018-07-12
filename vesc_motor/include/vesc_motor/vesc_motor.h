//
// Created by abuchegger on 04.07.18.
//
#ifndef VESC_MOTOR_VESC_MOTOR_H
#define VESC_MOTOR_VESC_MOTOR_H

#include <atomic>
#include <chrono>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_motor
{
class VescMotor
{
public:
  VescMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory,
            double execution_duration);
  virtual ~VescMotor() = default;

  /**
   * Gets the motor controller's supply voltage in V.
   * @return the supply voltage in V.
   */
  double getSupplyVoltage();

protected:
  void updateDriver(bool use_mockup);

  virtual void processMotorControllerState(const vesc_driver::MotorControllerState& state);

  boost::shared_ptr<vesc_driver::VescDriverInterface> driver_;

private:
  void callProcessMotorControllerState(const vesc_driver::MotorControllerState& state);

  ros::NodeHandle private_nh_;
  std::shared_ptr<VescTransportFactory> transport_factory_;
  std::chrono::duration<double> execution_duration_;
  vesc_driver::VescDriverInterface::StateHandlerFunction state_handler_function_;
  std::atomic<double> supply_voltage_;
};
}

#endif //VESC_MOTOR_VESC_MOTOR_H
