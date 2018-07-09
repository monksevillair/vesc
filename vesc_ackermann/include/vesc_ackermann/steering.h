//
// Created by abuchegger on 09.07.18.
//
#ifndef VESC_ACKERMANN_STEERING_H
#define VESC_ACKERMANN_STEERING_H

#include <vesc_ackermann/types.h>

namespace vesc_ackermann
{
/**
 * Representation of the steering geometry for one or more wheels.
 *
 * Instances are used to compute the wheel's steering angle and angular velocity from its steering motor's angle and
 * velocity.
 */
class Steering
{
public:
  virtual ~Steering() = default;

  virtual double computeWheelSteeringAngle(const Wheel& wheel, double steering_angle) const = 0;
  virtual double computeWheelSteeringVelocity(const Wheel& wheel, double steering_angle,
                                              double steering_velocity) const = 0;
};

/**
 * Computes the steering angle of the wheel from the steering angle of the hypothetical central wheel, assuming ideal
 * Ackermann geometry.
 */
class IdealAckermannSteering : public Steering
{
public:
  explicit IdealAckermannSteering(double icr_x);

  double computeWheelSteeringAngle(const Wheel& wheel, double steering_angle) const override;
  double computeWheelSteeringVelocity(const Wheel& wheel, double steering_angle,
                                      double steering_velocity) const override;

protected:
  double icr_x_;
};
}

#endif //VESC_ACKERMANN_STEERING_H
