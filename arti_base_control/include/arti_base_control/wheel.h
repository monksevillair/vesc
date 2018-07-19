//
// Created by abuchegger on 05.07.18.
//
#ifndef ARTI_BASE_CONTROL_WHEEL_H
#define ARTI_BASE_CONTROL_WHEEL_H

#include <arti_base_control/types.h>
#include <boost/optional.hpp>

namespace arti_base_control
{
/**
 * Representation of a wheel's and its steering hinge's geometry.
 *
 * The positions' coordinate frame origin (0, 0) is the (hypothetical) instant center of rotation when the vehicle is
 * rotating in place. The coordinate frame is furthermore aligned so that the instant center of rotation is always on
 * the line x == 0.
 */
class Wheel
{
public:
  Wheel() = default;
  Wheel(double position_x, double position_y, double hinge_position_y, double radius, const SteeringConstPtr& steering);

  /**
   * Computes the angular velocity (in rad/s) of the wheel given the intended movement of the vehicle.
   *
   * @param linear_velocity the intended linear velocity (in m/s) of the vehicle at the origin (0, 0).
   * @param angular_velocity the intended angular velocity (in rad/s) of the vehicle.
   * @param steering_angle the current steering angle (in rad) of a hypothetical wheel at position (position_x_, 0).
   *   This is only relevant if the wheel is steered and its hinge's position is different from its own position.
   * @param steering_velocity the rate of change (in rad/s) of the steering angle of a hypothetical wheel at
   *   (position_x_, 0). This is only relevant if the wheel is steered and its hinge's position is different from its
   *   own position.
   * @return the angular velocity (in rad/s) of the wheel.
   */
  double computeWheelVelocity(double linear_velocity, double angular_velocity, double steering_angle,
                              double steering_velocity) const;

  void computeVehicleVelocityConstraints(
    boost::optional<double> wheel_velocity, double steering_angle, double steering_velocity,
    VehicleVelocityConstraints& constraints) const;

  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double hinge_position_y_ = 0.0;
  double radius_ = 0.0;
  SteeringConstPtr steering_;
};
}

#endif //ARTI_BASE_CONTROL_WHEEL_H
