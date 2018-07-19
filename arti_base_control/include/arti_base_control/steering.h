//
// Created by abuchegger on 09.07.18.
//
#ifndef ARTI_BASE_CONTROL_STEERING_H
#define ARTI_BASE_CONTROL_STEERING_H

#include <arti_base_control/types.h>

namespace arti_base_control
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

  double icr_x_;
};
}

#endif //ARTI_BASE_CONTROL_STEERING_H
