//
// Created by abuchegger on 06.07.18.
//
#ifndef VESC_ACKERMANN_VEHICLE_VELOCITY_H
#define VESC_ACKERMANN_VEHICLE_VELOCITY_H

#include <boost/optional.hpp>

namespace vesc_ackermann
{
struct VehicleVelocity
{
  VehicleVelocity(double v_x_, double v_y_, double v_theta_);

  double v_x;
  double v_y;
  double v_theta;
};

struct VehicleVelocityConstraint
{
  VehicleVelocityConstraint() = default;
  VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_);

  double a_v_x = 0.0;
  double a_v_y = 0.0;
  double a_v_theta = 0.0;
  double b = 0.0;
};
}

#endif //VESC_ACKERMANN_VEHICLE_VELOCITY_H
