/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_SERIAL_PACKET_CODEC_H
#define VESC_DRIVER_SERIAL_PACKET_CODEC_H

#include <vesc_driver/packet.h>
#include <vesc_driver/byte_buffer.h>
#include <boost/optional.hpp>
#include <cstdint>
#include <boost/crc.hpp>

namespace vesc_driver
{
  class SerialPacketCodec
  {
  public:
    virtual void encode(const PacketVariant& packet, ByteBuffer& buffer)
    {
      Encoder encoder(buffer);
      boost::apply_visitor(encoder, packet);
    }

    virtual boost::optional<PacketVariant> decode(const ByteBuffer& buffer);

    constexpr static size_t VESC_MIN_FRAME_SIZE=5;
    constexpr static size_t VESC_SMALL_FRAME=2;
    constexpr static size_t VESC_LARGE_FRAME=3;
    constexpr static size_t VESC_EOF_BYTE=3;

    typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

  protected:
    class Encoder : public boost::static_visitor<>
    {
    public:
      Encoder(ByteBuffer& buffer) : buffer_(buffer) { }

      void operator()(const SetDutyCyclePacket& packet);
      void operator()(const SetCurrentPacket& packet);
      void operator()(const SetBrakePacket& packet);
      void operator()(const SetSpeedPacket& packet);
      void operator()(const SetPositionPacket& packet);
      void operator()(const GetValuesPacket& packet);
      void operator()(const MotorControllerState& packet);
      void operator()(const GetFirmwareVersion& packet);
      void operator()(const FirmwareVersion& packet);

      constexpr static uint8_t VESC_SET_DUTY_CYCLE_PACKET_PAYLOAD_SIZE=5;
      constexpr static uint8_t VESC_SET_DUTY_CYCLE_PACKET=5;

      constexpr static uint8_t VESC_SET_CURRENT_PACKET_PAYLOAD_SIZE=5;
      constexpr static uint8_t VESC_SET_CURRENT_PACKET=6;

      constexpr static uint8_t VESC_SET_BRAKE_PACKET_PAYLOAD_SIZE=5;
      constexpr static uint8_t VESC_SET_BRAKE_PACKET=7;

      constexpr static uint8_t VESC_SET_SPEED_PACKET_PAYLOAD_SIZE=5;
      constexpr static uint8_t VESC_SET_SPEED_PACKET=8;

      constexpr static uint8_t VESC_SET_POSITION_PACKET_PAYLOAD_SIZE=5;
      constexpr static uint8_t VESC_SET_POSITION_PACKET=9;

      constexpr static uint8_t VESC_GET_VALUES_PACKET_PAYLOAD_SIZE=1;
      constexpr static uint8_t VESC_GET_VALUES_POSITION_PACKET=4;

      constexpr static uint8_t VESC_GET_FW_VERSION_PACKET_PAYLOAD_SIZE=1;
      constexpr static uint8_t VESC_GET_FW_VERSION_POSITION_PACKET=0;
    protected:
      ByteBuffer& buffer_;
      uint16_t payload_size_;

      void createHeader(const uint16_t payload_size);

      void setPayloadID(uint8_t payload_id);

      void addCRC();
    };

    boost::optional<PacketVariant> decodeMotorControllerState(ByteBuffer &buffer);

    boost::optional<PacketVariant> decodeFirmwareVersion(ByteBuffer &buffer);
  };
}

#endif //VESC_DRIVER_SERIAL_PACKET_CODEC_H
