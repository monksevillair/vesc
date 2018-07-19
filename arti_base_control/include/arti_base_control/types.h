//
// Created by abuchegger on 09.07.18.
//
#ifndef ARTI_BASE_CONTROL_TYPES_H
#define ARTI_BASE_CONTROL_TYPES_H

#include <memory>
#include <vector>

namespace arti_base_control
{
class Axle;

typedef std::shared_ptr<Axle> AxlePtr;

class DriveMotor;

typedef std::shared_ptr<DriveMotor> DriveMotorPtr;

class MotorFactory;

typedef std::shared_ptr<MotorFactory> MotorFactoryPtr;

class Steering;

typedef std::shared_ptr<Steering const> SteeringConstPtr;

class SteeringMotor;

typedef std::shared_ptr<SteeringMotor> SteeringMotorPtr;

struct VehicleVelocityConstraint;

typedef std::vector<VehicleVelocityConstraint> VehicleVelocityConstraints;

class Wheel;

}

#endif //ARTI_BASE_CONTROL_TYPES_H
