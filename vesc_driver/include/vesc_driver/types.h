//
// Created by abuchegger on 19.07.18.
//
#ifndef VESC_DRIVER_TYPES_H
#define VESC_DRIVER_TYPES_H

#include <memory>

namespace vesc_driver
{
struct FirmwareVersion;

struct MotorControllerState;

class Transport;

typedef std::shared_ptr<Transport> TransportPtr;

class TransportRequest;

class VescDriverInterface;

typedef std::shared_ptr<VescDriverInterface> VescDriverInterfacePtr;
}

#endif //VESC_DRIVER_TYPES_H
