//
// Created by abuchegger on 09.07.18.
//
#include <arti_base_control/steering.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/wheel.h>
#include <cmath>

namespace arti_base_control
{
IdealAckermannSteering::IdealAckermannSteering(double icr_x)
  : icr_x_(icr_x)
{
}

double IdealAckermannSteering::computeWheelSteeringAngle(const Wheel& wheel, const double steering_angle) const
{
  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (wheel.position_x_ == icr_x_)
  {
    return 0.0;
  }

  const double sin_steering_angle = std::sin(steering_angle);
  const double x = wheel.position_x_ - icr_x_;
  return normalizeSteeringAngle(
    std::atan2(x * sin_steering_angle, x * std::cos(steering_angle) - wheel.hinge_position_y_ * sin_steering_angle));
}

double IdealAckermannSteering::computeWheelSteeringVelocity(const Wheel& wheel, double steering_angle,
                                                            double steering_velocity) const
{
  // In case the wheel is on the ICR line (which makes no sense for a steered wheel), default to no steering:
  if (wheel.position_x_ == icr_x_)
  {
    return 0.0;
  }

  // This is the time derivative of the above function:
  const double x = wheel.position_x_ - icr_x_;
  const double x2 = x * x;
  const double sin_steering_angle = std::sin(steering_angle);
  const double k = x * std::cos(steering_angle) - wheel.hinge_position_y_ * sin_steering_angle;

  return steering_velocity * x2 / (k * k + x2 * sin_steering_angle * sin_steering_angle);
}
}
