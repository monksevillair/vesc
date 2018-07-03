/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_SERIAL_PACKET_CODEC_H
#define VESC_DRIVER_SERIAL_PACKET_CODEC_H

#include <boost/optional.hpp>
#include <cstdint>
#include <vesc_driver/packet.h>
#include <vesc_driver/serial_data_reader.h>
#include <vesc_driver/serial_data_writer.h>

namespace vesc_driver
{
class SerialPacketCodec
{
public:
  void encode(SerialDataWriter& writer, const RequestPacket& packet);
  void encodeForwardCan(SerialDataWriter& writer, uint8_t controller_id, const RequestPacket& packet);

  boost::optional<ResponsePacket> decode(SerialDataReader& payload_reader);

protected:
  enum class PayloadId : uint8_t
  {
    GET_FIRMWARE_VERSION = 0,
    GET_VALUES = 4,
    SET_DUTY_CYCLE = 5,
    SET_CURRENT = 6,
    SET_BRAKE = 7,
    SET_SPEED = 8,
    SET_POSITION = 9,
    FORWARD_CAN = 34,
  };

  class Encoder : public boost::static_visitor<>
  {
  public:
    explicit Encoder(SerialDataWriter& writer);

    void operator()(const SetDutyCyclePacket& packet);
    void operator()(const SetCurrentPacket& packet);
    void operator()(const SetBrakePacket& packet);
    void operator()(const SetSpeedPacket& packet);
    void operator()(const SetPositionPacket& packet);
    void operator()(const GetValuesPacket& packet);
    void operator()(const GetFirmwareVersionPacket& packet);

  protected:
    SerialDataWriter& writer_;
  };

  static void writePayloadId(SerialDataWriter& writer, PayloadId payload_id);
  MotorControllerState decodeMotorControllerState(SerialDataReader& reader);
  FirmwareVersion decodeFirmwareVersion(SerialDataReader& reader);
};
}

#endif //VESC_DRIVER_SERIAL_PACKET_CODEC_H
