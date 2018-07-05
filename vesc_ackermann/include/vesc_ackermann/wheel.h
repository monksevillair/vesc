//
// Created by abuchegger on 05.07.18.
//
#ifndef VESC_ACKERMANN_WHEEL_H
#define VESC_ACKERMANN_WHEEL_H

namespace vesc_ackermann
{
/**
 * Representation of a wheel and its steering hinge.
 *
 * The positions' coordinate frame origin (0, 0) is the (hypothetical) instant center of rotation when the vehicle is
 * rotating in place. The coordinate frame is furthermore aligned so that the instant center of rotation is always on
 * the line x == 0.
 */
class Wheel
{
public:
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
  double computeVelocity(double linear_velocity, double angular_velocity, double steering_angle,
                         double steering_velocity) const;

  /**
   * Computes the steering angle of the wheel from the steering angle of the hypothetical central wheel, assuming ideal
   * Ackermann geometry.
   *
   * @param steering_angle the steering angle of the hypothetical central wheel.
   * @return the steering angle of the wheel.
   */
  double computeWheelSteeringAngle(double steering_angle) const;
  double computeWheelSteeringVelocity(double steering_angle, double steering_velocity) const;

  static constexpr double SIMULATION_TIME = 0.01;

  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double hinge_position_y_ = 0.0;
  double radius_ = 0.0;
};
}

#endif //VESC_ACKERMANN_WHEEL_H
