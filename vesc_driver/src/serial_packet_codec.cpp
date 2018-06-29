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
void SerialPacketCodec::encode(const PacketVariant& packet, ByteBuffer& buffer)
{
  ROS_DEBUG_STREAM("SerialPacketCodec::encode packet type: " << packet.type().name());
  Encoder encoder(buffer);
  boost::apply_visitor(encoder, packet);
}

boost::optional<vesc_driver::PacketVariant>
SerialPacketCodec::decode(const vesc_driver::ByteBuffer& buffer)
{
  ByteBuffer parsing_buffer = buffer;
  if (buffer.getSize() < 1)
  {
    ROS_ERROR_STREAM("SerialPacketCodec::decode : got empty buffer, that should never happen");
  }
  else
  {
    const uint8_t payload_id = parsing_buffer.parseUnsignedInt8();
    try
    {
      switch (payload_id)
      {
        case Encoder::VESC_GET_VALUES_POSITION_PACKET:
          ROS_DEBUG_STREAM("SerialPacketCodec::decode : Encoder::VESC_GET_VALUES_POSITION_PACKET");
          return decodeMotorControllerState(parsing_buffer);

        case Encoder::VESC_GET_FW_VERSION_PACKET:
          ROS_DEBUG_STREAM("SerialPacketCodec::decode : Encoder::VESC_GET_FW_VERSION_PACKET");
          return decodeFirmwareVersion(parsing_buffer);

        default:
          // we are not parsing any other packet
          ROS_DEBUG_STREAM("SerialPacketCodec::decode : other payload id: " << static_cast<unsigned>(payload_id));
          break;
      }
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR_STREAM("SerialPacketCodec::decode : failed to decode packet with payload id "
                         << static_cast<unsigned>(payload_id));
    }
  }
  return boost::none;
}

void SerialPacketCodec::forwardCan(const PacketVariant& packet, ByteBuffer& buffer, uint8_t can_id)
{
  Encoder encoder(buffer);
  boost::apply_visitor(encoder, packet);
  encoder.forwardCan(can_id);
}

PacketVariant SerialPacketCodec::decodeMotorControllerState(ByteBuffer& buffer)
{
  MotorControllerState result;
  buffer.advanceTo(5);
  result.current_motor = buffer.parseFloat32() / 1e2;
  result.current_input = buffer.parseFloat32() / 1e2;

  buffer.advanceTo(21);
  result.duty_cycle = buffer.parseFloat16() / 1e3;
  result.speed = buffer.parseFloat32() / 1e0;
  result.voltage_input = buffer.parseFloat16() / 1e1;
  result.charge_drawn = buffer.parseFloat32() / 1e4;
  result.charge_regen = buffer.parseFloat32() / 1e4;

  buffer.advanceTo(45);
  result.displacement = buffer.parseInt32();
  result.distance_traveled = buffer.parseInt32();
  result.fault_code = static_cast<MotorControllerState::FaultCode>(buffer.parseUnsignedInt8());

  return PacketVariant(result);
}

PacketVariant SerialPacketCodec::decodeFirmwareVersion(ByteBuffer& buffer)
{
  FirmwareVersion result;
  result.major_version = buffer.parseUnsignedInt8();
  result.minor_version = buffer.parseUnsignedInt8();
  return PacketVariant(result);
}

void SerialPacketCodec::Encoder::operator()(const SetDutyCyclePacket& packet)
{
  createHeader(VESC_SET_DUTY_CYCLE_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_SET_DUTY_CYCLE_PACKET);
  buffer_.addFloat32(packet.duty_cycle * 1e5);
  addCRC();
}

void SerialPacketCodec::Encoder::operator()(const SetCurrentPacket& packet)
{
  createHeader(VESC_SET_CURRENT_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_SET_CURRENT_PACKET);
  buffer_.addFloat32(packet.current * 1e3);
  addCRC();
}

void SerialPacketCodec::Encoder::operator()(const SetBrakePacket& packet)
{
  createHeader(VESC_SET_BRAKE_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_SET_BRAKE_PACKET);
  buffer_.addFloat32(packet.brake_current * 1e3);
  addCRC();
}

void SerialPacketCodec::Encoder::operator()(const SetSpeedPacket& packet)
{
  createHeader(VESC_SET_SPEED_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_SET_SPEED_PACKET);
  buffer_.addFloat32(packet.speed);
  addCRC();

  ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<SetSpeedPacket>");
  const auto encoded_bytes = static_cast<std::vector<uint8_t>>(buffer_);
  for (auto byte : encoded_bytes)
  {
    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<SetSpeedPacket> encoded_bytes: " << static_cast<int>(byte));
  }
}

void SerialPacketCodec::Encoder::operator()(const SetPositionPacket& packet)
{
  createHeader(VESC_SET_POSITION_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_SET_POSITION_PACKET);
  buffer_.addFloat32(packet.position * 1e6);
  addCRC();
}

void SerialPacketCodec::Encoder::operator()(const GetValuesPacket& packet)
{
  createHeader(VESC_GET_VALUES_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_GET_VALUES_POSITION_PACKET);
  addCRC();

  ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetValuesPacket>");
  const auto encoded_bytes = static_cast<std::vector<uint8_t>>(buffer_);
  for (auto byte : encoded_bytes)
  {
    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetValuesPacket> encoded_bytes: " << static_cast<int>(byte));
  }
}

void SerialPacketCodec::Encoder::operator()(const MotorControllerState& packet)
{
  throw std::invalid_argument("motor controller state cannot be encoded");
}

void SerialPacketCodec::Encoder::operator()(const GetFirmwareVersion& packet)
{
  createHeader(VESC_GET_FW_VERSION_PACKET_PAYLOAD_SIZE);
  setPayloadID(VESC_GET_FW_VERSION_PACKET);
  addCRC();

  ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetFirmwareVersion>");
  const auto encoded_bytes = static_cast<std::vector<uint8_t>>(buffer_);
  for (auto byte : encoded_bytes)
  {
    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetFirmwareVersion> encoded_bytes: "
                       << static_cast<int>(byte));
  }
}

void SerialPacketCodec::Encoder::operator()(const FirmwareVersion& packet)
{
  throw std::invalid_argument("firmware version can not be encoded");
}

void SerialPacketCodec::Encoder::createHeader(const uint16_t payload_size)
{
  if (payload_size < std::numeric_limits<uint8_t>::max())
  {
    buffer_.addUnsignedInt8(VESC_SMALL_FRAME);
    buffer_.addUnsignedInt8(static_cast<uint8_t>(payload_size));
  }
  else
  {
    buffer_.addUnsignedInt8(VESC_LARGE_FRAME);
    buffer_.addUnsignedInt16(payload_size);
  }
}

void SerialPacketCodec::Encoder::setPayloadID(uint8_t payload_id)
{
  buffer_.addUnsignedInt8(payload_id);
}

void SerialPacketCodec::Encoder::addCRC()
{
  buffer_.resetParsing();
  uint8_t start_byte = buffer_.parseUnsignedInt8();

  const size_t offset = (start_byte == VESC_SMALL_FRAME) ? 2 : 3;

  const auto bytes = static_cast<std::vector<uint8_t>>(buffer_);
  SerialPacketCodec::CRC crc_calculation;
  crc_calculation.process_bytes(&bytes.at(offset), bytes.size() - offset);
  const auto checksum = static_cast<uint16_t>(crc_calculation.checksum());
  buffer_.addUnsignedInt16(checksum);
  buffer_.addUnsignedInt8(VESC_EOF_BYTE);
}

void SerialPacketCodec::Encoder::forwardCan(uint8_t can_id)
{
  const auto encoded_bytes = static_cast<std::vector<uint8_t>>(buffer_);
  for (const auto byte : encoded_bytes)
  {
    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::fowardCanen::1 coded_bytes: " << static_cast<int>(byte));
  }

  buffer_.resetParsing();
  uint8_t start_byte = buffer_.parseUnsignedInt8();

  uint16_t payload_size;
  size_t offset = 1;
  if (start_byte == VESC_SMALL_FRAME)
  {
    payload_size = buffer_.parseUnsignedInt8();
    offset += 1;
  }
  else
  {
    payload_size = buffer_.parseUnsignedInt16();
    offset += 2;
  }

  auto original_bytes = static_cast<std::vector<uint8_t>>(buffer_);
  original_bytes.erase(original_bytes.begin(), original_bytes.begin() + offset);
  original_bytes.erase(original_bytes.begin() + payload_size, original_bytes.end());
  buffer_.clear();

  createHeader(payload_size + VESC_FORWARD_CAN_PAYLOAD_SIZE);
  setPayloadID(VESC_FORWARD_CAN);
  buffer_.addUnsignedInt8(can_id);
  buffer_.addBytes(original_bytes);
  addCRC();

  const auto encoded_bytes2 = static_cast<std::vector<uint8_t>>(buffer_);
  for (const auto byte : encoded_bytes2)
  {
    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::forwardCan::2 encoded_bytes: " << static_cast<int>(byte));
  }
}
}
