//
// Created by abuchegger on 06.07.18.
//
#include <vesc_ackermann/vehicle_velocity.h>

namespace vesc_ackermann
{
VehicleVelocity::VehicleVelocity(double linear_velocity_, double angular_velocity_)
  : linear_velocity(linear_velocity_), angular_velocity(angular_velocity_)
{
}
}
