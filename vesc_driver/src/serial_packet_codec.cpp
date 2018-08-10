/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/serial_packet_codec.h>
#include <ros/console.h>

namespace vesc_driver
{
void SerialPacketCodec::encode(SerialDataWriter& writer, const RequestPacket& packet)
{
  ROS_DEBUG_STREAM("SerialPacketCodec::encode: packet type: " << packet.type().name());
  Encoder encoder(writer);
  boost::apply_visitor(encoder, packet);
}

void SerialPacketCodec::encodeForwardCan(SerialDataWriter& writer, uint8_t controller_id, const RequestPacket& packet)
{
  ROS_DEBUG_STREAM("SerialPacketCodec::encodeForwardCan: packet type: " << packet.type().name());
  writePayloadId(writer, PayloadId::FORWARD_CAN);
  writer.writeUnsignedInt8(controller_id);
  Encoder encoder(writer);
  boost::apply_visitor(encoder, packet);
}

boost::optional<vesc_driver::ResponsePacket> SerialPacketCodec::decode(SerialDataReader& payload_reader)
{
  if (!payload_reader.canRead())
  {
    ROS_ERROR_STREAM("SerialPacketCodec::decode : got empty payload, that should never happen");
  }
  else
  {
    const uint8_t payload_id = payload_reader.readUnsignedInt8();
    try
    {
      switch (payload_id)
      {
        case static_cast<uint8_t>(PayloadId::GET_VALUES):
          ROS_DEBUG_STREAM("SerialPacketCodec::decode: PayloadId::GET_VALUES");
          return ResponsePacket(decodeMotorControllerState(payload_reader));

        case static_cast<uint8_t>(PayloadId::GET_FIRMWARE_VERSION):
          ROS_DEBUG_STREAM("SerialPacketCodec::decode: Encoder::VESC_GET_FW_VERSION_PACKET");
          return ResponsePacket(decodeFirmwareVersion(payload_reader));

        default:
          // we are not parsing any other packet
          ROS_DEBUG_STREAM("SerialPacketCodec::decode: other payload id: " << static_cast<unsigned>(payload_id));
          break;
      }
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR_STREAM("SerialPacketCodec::decode: failed to decode packet with payload id "
                         << static_cast<unsigned>(payload_id));
    }
  }
  return boost::none;
}

void SerialPacketCodec::writePayloadId(SerialDataWriter& writer, PayloadId payload_id)
{
  writer.writeUnsignedInt8(static_cast<uint8_t>(payload_id));
}

MotorControllerState SerialPacketCodec::decodeMotorControllerState(SerialDataReader& reader)
{
  MotorControllerState result;
  reader.skipBytes(4);
  result.current_motor = reader.readFloat32() / 1e2;
  result.current_input = reader.readFloat32() / 1e2;

  reader.skipBytes(8);
  result.duty_cycle = reader.readFloat16() / 1e3;
  result.speed = reader.readFloat32() / 1e0;
  result.voltage_input = reader.readFloat16() / 1e1;
  result.charge_drawn = reader.readFloat32() / 1e4;
  result.charge_regen = reader.readFloat32() / 1e4;

  reader.skipBytes(8);
  result.displacement = reader.readInt32();
  result.distance_traveled = reader.readInt32();
  result.fault_code = static_cast<MotorControllerState::FaultCode>(reader.readUnsignedInt8());
  result.position = reader.readFloat32() / 1e6;

  return result;
}

FirmwareVersion SerialPacketCodec::decodeFirmwareVersion(SerialDataReader& reader)
{
  FirmwareVersion result;
  result.major_version = reader.readUnsignedInt8();
  result.minor_version = reader.readUnsignedInt8();
  return result;
}

SerialPacketCodec::Encoder::Encoder(SerialDataWriter& writer)
  : writer_(writer)
{
}

void SerialPacketCodec::Encoder::operator()(const SetDutyCyclePacket& packet)
{
  writePayloadId(writer_, PayloadId::SET_DUTY_CYCLE);
  writer_.writeFloat32(packet.duty_cycle * 1e5);
}

void SerialPacketCodec::Encoder::operator()(const SetCurrentPacket& packet)
{
  writePayloadId(writer_, PayloadId::SET_CURRENT);
  writer_.writeFloat32(packet.current * 1e3);
}

void SerialPacketCodec::Encoder::operator()(const SetBrakePacket& packet)
{
  writePayloadId(writer_, PayloadId::SET_BRAKE);
  writer_.writeFloat32(packet.brake_current * 1e3);
}

void SerialPacketCodec::Encoder::operator()(const SetSpeedPacket& packet)
{
  writePayloadId(writer_, PayloadId::SET_SPEED);
  writer_.writeFloat32(packet.speed);
}

void SerialPacketCodec::Encoder::operator()(const SetPositionPacket& packet)
{
  writePayloadId(writer_, PayloadId::SET_POSITION);
  writer_.writeFloat32(packet.position * 1e6);
}

void SerialPacketCodec::Encoder::operator()(const GetValuesPacket& /*packet*/)
{
  writePayloadId(writer_, PayloadId::GET_VALUES);
}

void SerialPacketCodec::Encoder::operator()(const GetFirmwareVersionPacket& /*packet*/)
{
  writePayloadId(writer_, PayloadId::GET_FIRMWARE_VERSION);
}
}
