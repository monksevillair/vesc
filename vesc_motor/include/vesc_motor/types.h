//
// Created by abuchegger on 19.07.18.
//
#ifndef VESC_MOTOR_TYPES_H
#define VESC_MOTOR_TYPES_H

#include <memory>

namespace vesc_motor
{
class DriverFactory;
typedef std::shared_ptr<DriverFactory> DriverFactoryPtr;

class TransportFactory;
typedef std::shared_ptr<TransportFactory> TransportFactoryPtr;

class VescDriveMotor;
typedef std::shared_ptr<VescDriveMotor> VescDriveMotorPtr;

class VescMotor;
typedef std::shared_ptr<VescMotor> VescMotorPtr;

class VescSteeringMotor;
typedef std::shared_ptr<VescSteeringMotor> VescSteeringMotorPtr;
}

#endif //VESC_MOTOR_TYPES_H
