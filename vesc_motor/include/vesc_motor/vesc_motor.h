//
// Created by abuchegger on 04.07.18.
//
#ifndef VESC_MOTOR_VESC_MOTOR_H
#define VESC_MOTOR_VESC_MOTOR_H

#include <atomic>
#include <chrono>
#include <ros/node_handle.h>
#include <vesc_driver/types.h>
#include <vesc_motor/types.h>

namespace vesc_motor
{
class VescMotor
{
public:
  VescMotor(const ros::NodeHandle& private_nh, const DriverFactoryPtr& driver_factory,
            const std::chrono::duration<double>& execution_duration);
  virtual ~VescMotor() = default;

  /**
   * Gets the motor controller's supply voltage in V.
   * @return the supply voltage in V.
   */
  double getSupplyVoltage();

protected:
  void createDriver();
  virtual void processMotorControllerState(const vesc_driver::MotorControllerState& state) = 0;

  ros::NodeHandle private_nh_;
  vesc_driver::VescDriverInterfacePtr driver_;

private:
  void callProcessMotorControllerState(const vesc_driver::MotorControllerState& state);

  DriverFactoryPtr driver_factory_;
  std::chrono::duration<double> execution_duration_;
  std::atomic<double> supply_voltage_;
};
}

#endif //VESC_MOTOR_VESC_MOTOR_H
