//
// Created by abuchegger on 05.07.18.
//
#include <arti_base_control/wheel.h>
#include <angles/angles.h>
#include <arti_base_control/steering.h>
#include <arti_base_control/vehicle.h>
#include <cmath>
#include <stdexcept>

namespace arti_base_control
{
Wheel::Wheel(const double position_x, const double position_y, const double hinge_position_y, const double radius,
             const SteeringConstPtr& steering)
  : position_x_(position_x), position_y_(position_y), hinge_position_y_(hinge_position_y), radius_(radius),
    steering_(steering)
{
}

double Wheel::computeWheelVelocity(const double linear_velocity, const double angular_velocity,
                                   const double steering_angle, const double steering_velocity) const
{
  if (position_x_ == 0.0)
  {
    // If the wheel is not steered, the formula becomes very simple:
    return (linear_velocity - angular_velocity * position_y_) / radius_;
  }

  const double wheel_steering_velocity
    = steering_ ? steering_->computeWheelSteeringVelocity(*this, steering_angle, steering_velocity) : 0.0;

  const double tangential_velocity
    = std::hypot(linear_velocity - angular_velocity * hinge_position_y_, angular_velocity * position_x_)
      * (linear_velocity < 0.0 ? -1.0 : 1.0)
      + (angular_velocity + wheel_steering_velocity) * (hinge_position_y_ - position_y_);

  return tangential_velocity / radius_;
}

void Wheel::computeVehicleVelocityConstraints(
  boost::optional<double> wheel_velocity, double steering_angle, double steering_velocity,
  VehicleVelocityConstraints& constraints) const
{
  const double wheel_steering_angle = steering_ ? steering_->computeWheelSteeringAngle(*this, steering_angle) : 0.0;
  const double sin_sa = std::sin(wheel_steering_angle);
  const double cos_sa = std::cos(wheel_steering_angle);

  VehicleVelocityConstraint normal_velocity_constraint;
  normal_velocity_constraint.a_v_x = -sin_sa;
  normal_velocity_constraint.a_v_y = cos_sa;
  normal_velocity_constraint.a_v_theta = sin_sa * hinge_position_y_ + cos_sa * position_x_;
  normal_velocity_constraint.b = 0.0;
  constraints.push_back(normal_velocity_constraint);

  if (wheel_velocity)
  {
    const double wheel_steering_velocity
      = steering_ ? steering_->computeWheelSteeringVelocity(*this, steering_angle, steering_velocity) : 0.0;

    const double corrected_tangential_velocity
      = *wheel_velocity * radius_ - wheel_steering_velocity * (hinge_position_y_ - position_y_);

    VehicleVelocityConstraint tangential_velocity_constraint;
    tangential_velocity_constraint.a_v_x = cos_sa;
    tangential_velocity_constraint.a_v_y = sin_sa;
    tangential_velocity_constraint.a_v_theta
      = -cos_sa * hinge_position_y_ + sin_sa * position_x_ - position_y_ + hinge_position_y_;
    tangential_velocity_constraint.b = corrected_tangential_velocity;
    constraints.push_back(tangential_velocity_constraint);
  }
}
}
