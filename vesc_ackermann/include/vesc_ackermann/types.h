//
// Created by abuchegger on 09.07.18.
//
#ifndef VESC_ACKERMANN_TYPES_H
#define VESC_ACKERMANN_TYPES_H

#include <memory>
#include <vector>

namespace vesc_ackermann
{
class Axle;

typedef std::shared_ptr<Axle> AxlePtr;

class Steering;

typedef std::shared_ptr<Steering const> SteeringConstPtr;

struct VehicleVelocityConstraint;

typedef std::vector<VehicleVelocityConstraint> VehicleVelocityConstraints;

class Wheel;

}

#endif //VESC_ACKERMANN_TYPES_H
