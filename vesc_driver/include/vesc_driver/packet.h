/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_PACKET_H
#define VESC_DRIVER_PACKET_H

#include <boost/variant.hpp>
#include <vesc_driver/motor_controller_state.h>

namespace vesc_driver
{
  struct SetDutyCyclePacket
  {
    double duty_cycle;
  };

  struct SetCurrentPacket
  {
    double current;
  };

  struct SetBrakePacket
  {
    double brake_current;
  };

  struct SetSpeedPacket
  {
    double speed;
  };

  struct SetPositionPacket
  {
    double position;
  };

  struct GetValuesPacket
  {
  };

  struct GetFirmwareVersion
  {
  };

  struct FirmwareVersion
  {
    uint8_t major_version;
    uint8_t minor_version;
  };

  typedef boost::variant<SetDutyCyclePacket, SetCurrentPacket, SetBrakePacket, SetSpeedPacket, SetPositionPacket,
      GetValuesPacket, MotorControllerState, GetFirmwareVersion, FirmwareVersion> PacketVariant;
}

#endif //VESC_DRIVER_PACKET_H
