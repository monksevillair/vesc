/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/serial_packet_codec.h>
#include <ros/ros.h>

namespace vesc_driver
{
  boost::optional<vesc_driver::PacketVariant>
  SerialPacketCodec::decode(const vesc_driver::ByteBuffer &buffer)
  {
    if (buffer.getSize() < 1)
      return boost::none;

    ByteBuffer parsing_buffer = buffer;
    uint8_t payload_id = parsing_buffer.parsUnsignedInt8();
    ROS_DEBUG_STREAM("SerialPacketCodec::decode : payload_id: " << static_cast<int>(payload_id));    
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
        break;
    }

    return boost::none;
  }

  boost::optional<PacketVariant> SerialPacketCodec::decodeMotorControllerState(ByteBuffer &buffer)
  {
    if (buffer.getSize() < 53)
      return boost::none;

    MotorControllerState result;
    buffer.advanceTo(5);
    result.current_motor = buffer.parsFloat32() / 1e2;
    result.current_input = buffer.parsFloat32() / 1e2;

    buffer.advanceTo(21);
    result.duty_cycle = buffer.parsFloat16() / 1e3;
    result.speed = buffer.parsFloat32() / 1e0;
    result.voltage_input = buffer.parsFloat16() / 1e1;
    result.charge_drawn = buffer.parsFloat32() / 1e4;
    result.charge_regen = buffer.parsFloat32() / 1e4;

    buffer.advanceTo(45);
    result.displacement = buffer.parsInt32();
    result.distance_traveled = buffer.parsInt32();
    result.fault_code = MotorControllerState::FAULT_CODE(buffer.parsUnsignedInt8());

    return PacketVariant(result);
  }

  boost::optional<PacketVariant> SerialPacketCodec::decodeFirmwareVersion(ByteBuffer &buffer)
  {
    if (buffer.getSize() < 2)
      return boost::none;

    FirmwareVersion result;
    result.major_version = buffer.parsUnsignedInt8();
    result.minor_version = buffer.parsUnsignedInt8();

    return PacketVariant(result);
  }

  void SerialPacketCodec::Encoder::operator()(const SetDutyCyclePacket &packet)
  {
    createHeader(VESC_SET_DUTY_CYCLE_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_SET_DUTY_CYCLE_PACKET);
    buffer_.addFloat32(static_cast<float>(packet.duty_cycle) * 100000.0f);
    addCRC();
  }

  void SerialPacketCodec::Encoder::operator()(const SetCurrentPacket &packet)
  {
    createHeader(VESC_SET_CURRENT_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_SET_CURRENT_PACKET);
    buffer_.addFloat32(static_cast<float>(packet.current) * 1000.0f);
    addCRC();
  }

  void SerialPacketCodec::Encoder::operator()(const SetBrakePacket &packet)
  {
    createHeader(VESC_SET_BRAKE_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_SET_BRAKE_PACKET);
    buffer_.addFloat32(static_cast<float>(packet.brake_current) * 1000.0f);
    addCRC();
  }

  void SerialPacketCodec::Encoder::operator()(const SetSpeedPacket &packet)
  {
    createHeader(VESC_SET_SPEED_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_SET_SPEED_PACKET);
    buffer_.addFloat32(static_cast<float>(packet.speed));
    addCRC();

    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<SetSpeedPacket>");

    std::vector<uint8_t> encoded_bytes = buffer_;
    for (auto byte : encoded_bytes)
          ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<SetSpeedPacket> encoded_bytes: " << static_cast<int>(byte));     
  }

  void SerialPacketCodec::Encoder::operator()(const SetPositionPacket &packet)
  {
    createHeader(VESC_SET_POSITION_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_SET_POSITION_PACKET);
    buffer_.addFloat32(static_cast<float>(packet.position) * 1000000.0f);
    addCRC();
  }

  void SerialPacketCodec::Encoder::operator()(const GetValuesPacket &packet)
  {
    createHeader(VESC_GET_VALUES_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_GET_VALUES_POSITION_PACKET);
    addCRC();

    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetValuesPacket>");

    std::vector<uint8_t> encoded_bytes = buffer_;
    for (auto byte : encoded_bytes)
          ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetValuesPacket> encoded_bytes: " << static_cast<int>(byte)); 
  }

  void SerialPacketCodec::Encoder::operator()(const MotorControllerState &packet)
  {
    throw std::runtime_error("motor controller state can not be encoded just decoded");
  }

  void SerialPacketCodec::Encoder::operator()(const GetFirmwareVersion &packet)
  {
    createHeader(VESC_GET_FW_VERSION_PACKET_PAYLOAD_SIZE);
    setPayloadID(VESC_GET_FW_VERSION_PACKET);
    addCRC();

    ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetFirmwareVersion>");    

    std::vector<uint8_t> encoded_bytes = buffer_;
    for (auto byte : encoded_bytes)
          ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::operator<GetFirmwareVersion> encoded_bytes: " << static_cast<int>(byte));     
  }

  void SerialPacketCodec::Encoder::operator()(const FirmwareVersion &packet)
  {
    throw std::runtime_error("firmware version can not be encoded just decoded");
  }

  void SerialPacketCodec::Encoder::createHeader(const uint16_t payload_size)
  {
    if (payload_size < std::numeric_limits<uint8_t>::max())
    {
      buffer_.addUnsigedInt8(VESC_SMALL_FRAME);
      buffer_.addUnsigedInt8(static_cast<uint8_t>(payload_size));
    }
    else
    {
      buffer_.addUnsigedInt8(VESC_LARGE_FRAME);
      buffer_.addUnsigedInt16(payload_size);
    }

    payload_size_ = payload_size;
  }

  void SerialPacketCodec::Encoder::setPayloadID(uint8_t payload_id)
  {
    buffer_.addUnsigedInt8(payload_id);
  }

  void SerialPacketCodec::Encoder::addCRC()
  {
    buffer_.resetParsing();
    uint8_t start_byte = buffer_.parsUnsignedInt8();

    size_t offset = 1;
    if (start_byte == VESC_SMALL_FRAME)
      offset += 1;
    else
      offset += 2;

    std::vector<uint8_t> bytes = buffer_;
    SerialPacketCodec::CRC crc_calculation;
    crc_calculation.process_bytes(&(*(bytes.begin() + offset)), std::distance(bytes.begin() + offset, bytes.end()));
    uint16_t checksum = static_cast<uint16_t>(crc_calculation.checksum());
    buffer_.addUnsigedInt16(checksum);
    buffer_.addUnsigedInt8(VESC_EOF_BYTE);
  }

  void SerialPacketCodec::Encoder::fowardCan(uint8_t can_id)
  {
    std::vector<uint8_t> encoded_bytes = buffer_;
    for (auto byte : encoded_bytes)
          ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::fowardCanen::1 coded_bytes: " << static_cast<int>(byte)); 

    buffer_.resetParsing();
    uint8_t start_byte = buffer_.parsUnsignedInt8();

    uint16_t payload_size;
    size_t offset = 1;
    if (start_byte == VESC_SMALL_FRAME)
    {
      payload_size = buffer_.parsUnsignedInt8();
      offset += 1;
    }
    else
    {
      payload_size = buffer_.parsUnsignedInt16();
      offset += 2;
    }

    std::vector<uint8_t> original_bytes = buffer_;
    original_bytes.erase(original_bytes.begin(), original_bytes.begin() + offset);
    original_bytes.erase(original_bytes.begin() + payload_size, original_bytes.end());
    buffer_.clear();

    createHeader(payload_size + VESC_FOWARD_CAN_PAYLOAD_SIZE);
    setPayloadID(VESC_FOWARD_CAN);
    buffer_.addUnsigedInt8(can_id);
    buffer_.addBytes(original_bytes);
    addCRC();

    encoded_bytes = buffer_;
    for (auto byte : encoded_bytes)
          ROS_DEBUG_STREAM("SerialPacketCodec::Encoder::fowardCan::2 encoded_bytes: " << static_cast<int>(byte));     
  }

}