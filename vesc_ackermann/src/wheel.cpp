//
// Created by abuchegger on 05.07.18.
//
#include <vesc_ackermann/wheel.h>
#include <angles/angles.h>
#include <cmath>

namespace vesc_ackermann
{
double Wheel::computeVelocity(const double linear_velocity, const double angular_velocity, const double steering_angle,
                              const double steering_velocity) const
{
  if (position_x_ == 0.0)
  {
    // If the wheel is not steered, the formula becomes very simple:
    return (linear_velocity - angular_velocity * position_y_) / radius_;
  }

  const double wheel_steering_velocity = computeWheelSteeringVelocity(steering_angle, steering_velocity);

  const double tangential_velocity
    = std::hypot(linear_velocity - angular_velocity * hinge_position_y_, angular_velocity * position_x_)
      * (linear_velocity < 0.0 ? -1.0 : 1.0)
      + (angular_velocity + wheel_steering_velocity) * (hinge_position_y_ - position_y_);

  return tangential_velocity / radius_;
}

double Wheel::computeWheelSteeringAngle(const double steering_angle) const
{
  const double tan_steering_angle = std::tan(steering_angle);
  return std::atan2(position_x_ * tan_steering_angle, position_x_ - hinge_position_y_ * tan_steering_angle);
}

double Wheel::computeWheelSteeringVelocity(const double steering_angle, const double steering_velocity) const
{
  if (steering_velocity == 0.0)
  {
    return 0.0;
  }

  const double wheel_steering_angle = computeWheelSteeringAngle(steering_angle);
  const double new_steering_angle = steering_angle + steering_velocity * SIMULATION_TIME;
  const double new_wheel_steering_angle = computeWheelSteeringAngle(new_steering_angle);
  return angles::normalize_angle(new_wheel_steering_angle - wheel_steering_angle) / SIMULATION_TIME;
}
}
