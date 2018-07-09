//
// Created by abuchegger on 06.07.18.
//
#include <vesc_ackermann/vehicle_velocity.h>

namespace vesc_ackermann
{
VehicleVelocity::VehicleVelocity(double v_x_, double v_y_, double v_theta_)
  : v_x(v_x_), v_y(v_y_), v_theta(v_theta_)
{
}

VehicleVelocityConstraint::VehicleVelocityConstraint(double a_v_x_, double a_v_y_, double a_v_theta_, double b_)
  : a_v_x(a_v_x_), a_v_y(a_v_y_), a_v_theta(a_v_theta_), b(b_)
{
}
}
