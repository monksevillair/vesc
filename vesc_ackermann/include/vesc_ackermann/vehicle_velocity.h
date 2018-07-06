//
// Created by abuchegger on 06.07.18.
//
#ifndef VESC_ACKERMANN_VEHICLE_VELOCITY_H
#define VESC_ACKERMANN_VEHICLE_VELOCITY_H

namespace vesc_ackermann
{
struct VehicleVelocity
{
  VehicleVelocity(double linear_velocity_, double angular_velocity_);

  double linear_velocity;
  double angular_velocity;
};
}

#endif //VESC_ACKERMANN_VEHICLE_VELOCITY_H
