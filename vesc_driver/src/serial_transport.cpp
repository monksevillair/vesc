/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/serial_transport.h>
#include <ros/console.h>
#include <stdexcept>
#include <vesc_driver/serial_data_writer.h>

namespace vesc_driver
{

SerialTransport::SerialTransport(uint8_t controller_id, const std::string& port)
  : controller_id_(controller_id),
    serial_port_(std::string(), 115200, serial::Timeout::simpleTimeout(100), serial::eightbits, serial::parity_none,
                 serial::stopbits_one, serial::flowcontrol_none)
{
  // Must not set port in Serial constructor because that would open the port:
  serial_port_.setPort(port);
}

SerialTransport::~SerialTransport()
{
  disconnect();
}

void SerialTransport::submit(TransportRequest&& r)
{
  ROS_DEBUG_STREAM("SerialTransport::submit packet type: " << r.getPacket().type().name());

  write_queue_.push(std::move(r));
}

void SerialTransport::connect()
{
  if (isConnected())
  {
    throw std::logic_error("Tried to connect while already connected");
  }

  stopThreads();

  try
  {
    serial_port_.open();
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Encountered error on opening port '" + serial_port_.getPort() + "': " + e.what());
  }

  should_read_ = true;
  read_thread_ = std::thread(&SerialTransport::readLoop, this);

  should_write_ = true;
  write_thread_ = std::thread(&SerialTransport::writeLoop, this);
}

bool SerialTransport::isConnected()
{
  return serial_port_.isOpen();
}

void SerialTransport::disconnect()
{
  if (isConnected())
  {
    stopThreads();
    serial_port_.close();
  }
}

void SerialTransport::write(const Buffer& buffer)
{
  if (isConnected())
  {
    const size_t bytes_written = serial_port_.write(&buffer.at(0), buffer.size());

    if (bytes_written != buffer.size())
    {
      throw std::runtime_error("Could not write the complete buffer");
    }
  }
  else
  {
    throw std::runtime_error("Tried to write to closed serial port");
  }
}

void SerialTransport::readLoop()
{
  ROS_DEBUG_STREAM("SerialTransport::readLoop::1");

  while (should_read_)
  {
    ROS_DEBUG_STREAM("SerialTransport::readLoop::2");

    Buffer read_buffer;
    const uint8_t start_byte = readStartByte(read_buffer);
    if (!should_read_)
    {
      break;
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::3");

    ROS_DEBUG_STREAM("SerialTransport::readLoop::4");

    size_t payload_size_bytes;
    if (start_byte == SMALL_FRAME_SOF_BYTE)
    {
      payload_size_bytes = 1;
    }
    else if (start_byte == LARGE_FRAME_SOF_BYTE)
    {
      payload_size_bytes = 2;
    }
    else
    {
      ROS_DEBUG_STREAM("SerialTransport::readLoop::5");
      // Skip invalid packet:
      continue;
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::6 payload_size_bytes: " << payload_size_bytes);

    // At least start byte and payload size needs to be read:
    readBytesUntilSizeReached(1 + payload_size_bytes, read_buffer);
    if (!should_read_)
    {
      break;
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::7");

    uint16_t payload_size = 0;
    {
      SerialDataReader reader(read_buffer.begin() + 1, read_buffer.end());
      if (start_byte == SMALL_FRAME_SOF_BYTE)
      {
        payload_size = reader.readUnsignedInt8();
      }
      else if (start_byte == LARGE_FRAME_SOF_BYTE)
      {
        payload_size = reader.readUnsignedInt16();
      }
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::8 payload_size: " << static_cast<int>(payload_size));

    // At least start byte, payload size, payload, crc, and eof byte needs to be read:
    readBytesUntilSizeReached(1 + payload_size_bytes + payload_size + 2 + 1, read_buffer);
    if (!should_read_)
    {
      break;
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::9");
    ROS_DEBUG_STREAM("SerialTransport::readLoop::9.1 payload_size_bytes: "
                       << payload_size_bytes << " payload_size: " << payload_size << " read_buffer.size()"
                       << read_buffer.size());
    for (const auto byte : read_buffer)
    {
      ROS_DEBUG_STREAM("SerialTransport::readLoop::9.2 read_bytes: " << static_cast<int>(byte));
    }

    const Buffer::const_iterator payload_begin(read_buffer.begin() + 1 + payload_size_bytes);
    const Buffer::const_iterator payload_end(payload_begin + payload_size);

    {
      SerialDataReader reader(payload_end, read_buffer.end());
      const uint16_t crc_value = reader.readUnsignedInt16();
      const uint8_t eof_byte = reader.readUnsignedInt8();

      if (eof_byte != EOF_BYTE)
      {
        ROS_DEBUG_STREAM("SerialTransport::readLoop::10");
        // Skip invalid packet:
        continue;
      }

      ROS_DEBUG_STREAM("SerialTransport::readLoop::11");
      ROS_DEBUG_STREAM("SerialTransport::readLoop::12");

      CRC crc_calculation;
      crc_calculation.process_block(&*payload_begin, &*payload_end);
      if (crc_calculation.checksum() != crc_value)
      {
        ROS_DEBUG_STREAM("SerialTransport::readLoop::13");
        // Skip invalid packet:
        continue;
      }
    }

    ROS_DEBUG_STREAM("SerialTransport::readLoop::14");

    SerialDataReader payload_reader(payload_begin, payload_end);
    const boost::optional<ResponsePacket> packet = packet_codec_.decode(payload_reader);

    if (!packet)
    {
      ROS_DEBUG_STREAM("SerialTransport::readLoop::15");
      // Skip invalid packet:
      continue;
    }

    {
      ROS_DEBUG_STREAM("SerialTransport::readLoop::16");

      std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);
      if (should_respond_)
      {
        ROS_DEBUG_STREAM("SerialTransport::readLoop::17");

        should_respond_ = false;

        std::lock_guard<std::mutex> packet_handler_lock(packet_handler_mutex_);
        const PacketHandlers::const_iterator packet_handler_mapping = packet_handlers_.find(respond_id_);
        if (packet_handler_mapping != packet_handlers_.end())
        {
          ROS_DEBUG_STREAM("SerialTransport::readLoop::19");
          packet_handler_mapping->second(packet.get());
        }

        should_respond_condition_.notify_all();
      }
    }
  }
}

void SerialTransport::registerPacketHandler(uint8_t controller_id, Transport::PacketHandler&& packet_handler)
{
  std::lock_guard<std::mutex> pacekt_handler_lock(packet_handler_mutex_);
  packet_handlers_[controller_id] = packet_handler;
}

void SerialTransport::writeLoop()
{
  while (should_write_)
  {
    try
    {
      ROS_DEBUG_STREAM("SerialTransport::writeLoop::1");

      TransportRequest request = write_queue_.pop();
      ROS_DEBUG_STREAM("SerialTransport::writeLoop packet type: " << request.getPacket().type().name());

      ROS_DEBUG_STREAM("SerialTransport::writeLoop::2");

      {
        std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);
        while (should_respond_)
        {
          should_respond_condition_.wait(should_respond_lock);
        }
      }

      ROS_DEBUG_STREAM("SerialTransport::writeLoop::3");

      const Buffer frame_buffer = encodePacket(request);

      {
        std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);

        ROS_DEBUG_STREAM("SerialTransport::writeLoop::5");

        write(frame_buffer);

        ROS_DEBUG_STREAM("SerialTransport::writeLoop::6");

        if (request.expectResponse())
        {

          ROS_DEBUG_STREAM("SerialTransport::writeLoop::7");

          should_respond_ = true;
          respond_id_ = request.getControllerId();

          ROS_DEBUG_STREAM("SerialTransport::writeLoop::8");
        }
      }
    }
    catch (const InterruptException& exception)
    {
      ROS_DEBUG_STREAM("SerialTransport::writeLoop::9");
      break;
    }
  }
}

void SerialTransport::stopThreads()
{
  if (should_write_.exchange(false) && write_thread_.joinable())
  {
    write_queue_.interrupt();
    write_thread_.join();
  }

  if (should_read_.exchange(false) && read_thread_.joinable())
  {
    read_thread_.join();
  }
}

uint8_t SerialTransport::readStartByte(Buffer& buffer)
{
  while (should_read_)
  {
    ROS_DEBUG_STREAM("SerialTransport::readStartByte::1");

    buffer.clear();

    readBytes(MIN_FRAME_SIZE, buffer);
    ROS_DEBUG_STREAM("SerialTransport::readStartByte::2");

    for (Buffer::const_iterator it = buffer.begin(); it != buffer.end(); ++it)
    {
      const uint8_t start_byte = *it;
      if ((start_byte == SMALL_FRAME_SOF_BYTE) || (start_byte == LARGE_FRAME_SOF_BYTE))
      {
        ROS_DEBUG_STREAM("SerialTransport::readStartByte::5");

        // Remove bytes before start byte:
        buffer.erase(buffer.begin(), it);
        return start_byte;
      }
    }
  }
  return 0;
}

void SerialTransport::readBytes(const size_t size, Buffer& buffer)
{
  ROS_DEBUG_STREAM("SerialTransport::readBytes::1");
  ROS_DEBUG_STREAM("SerialTransport::readBytes::1.1 size: " << size);

  ROS_DEBUG_STREAM("SerialTransport::readBytes::2");

  const size_t start_index = buffer.size();
  buffer.resize(start_index + size);
  size_t read_bytes = 0;
  while (should_read_ && read_bytes < size)
  {
    ROS_DEBUG_STREAM("SerialTransport::readBytes::3");

    const size_t remaining_bytes = size - read_bytes;

    ROS_DEBUG_STREAM("SerialTransport::readBytes::4 remaining_bytes: " << remaining_bytes);

    // Using pointer variant of read() because implementation does not allocate extra buffer:
    const size_t new_read_bytes = serial_port_.read(&buffer.at(start_index + read_bytes), remaining_bytes);

    ROS_DEBUG_STREAM("SerialTransport::readBytes::5 new_read_bytes: " << new_read_bytes);

    ROS_DEBUG_STREAM("SerialTransport::readBytes::6");
    read_bytes += new_read_bytes;

    ROS_DEBUG_STREAM("SerialTransport::readBytes::7 read_bytes: " << read_bytes);
  }

  buffer.resize(start_index + read_bytes);

  ROS_DEBUG_STREAM("SerialTransport::readBytes::8");
  for (auto byte : buffer)
  {
    ROS_DEBUG_STREAM("SerialTransport::readBytes::8.1 buffer: " << static_cast<int>(byte));
  }
}

void SerialTransport::readBytesUntilSizeReached(const size_t size, Buffer& buffer)
{
  if (size > buffer.size())
  {
    readBytes(size - buffer.size(), buffer);
  }
}

SerialTransport::Buffer SerialTransport::encodePacket(const TransportRequest& request)
{
  Buffer payload_buffer;
  SerialDataWriter writer(payload_buffer);

  ROS_DEBUG_STREAM("SerialTransport::writeLoop::4");

  if (request.getControllerId() == controller_id_)
  {
    packet_codec_.encode(writer, request.getPacket());
  }
  else
  {
    packet_codec_.encodeForwardCan(writer, request.getControllerId(), request.getPacket());
  }

  for (const auto byte : payload_buffer)
  {
    ROS_DEBUG_STREAM("SerialTransport::writeLoop payload_buffer: " << static_cast<unsigned>(byte));
  }

  return addFrame(payload_buffer);
}

SerialTransport::Buffer SerialTransport::addFrame(const Buffer& payload)
{
  const size_t payload_size = payload.size();

  Buffer buffer;
  buffer.reserve(payload_size + MIN_LARGE_FRAME_SIZE);
  SerialDataWriter writer(buffer);

  if (payload_size < std::numeric_limits<uint8_t>::max())
  {
    writer.writeUnsignedInt8(SMALL_FRAME_SOF_BYTE);
    writer.writeUnsignedInt8(static_cast<uint8_t>(payload_size));
  }
  else if (payload_size < std::numeric_limits<uint16_t>::max())
  {
    writer.writeUnsignedInt8(LARGE_FRAME_SOF_BYTE);
    writer.writeUnsignedInt16(static_cast<uint16_t>(payload_size));
  }
  else
  {
    throw std::invalid_argument("packet payload is too large (" + std::to_string(payload_size) + " bytes)");
  }

  writer.writeBlock(payload.begin(), payload.end());

  CRC crc_calculation;
  crc_calculation.process_bytes(payload.data(), payload_size);
  writer.writeUnsignedInt16(static_cast<uint16_t>(crc_calculation.checksum()));
  writer.writeUnsignedInt8(EOF_BYTE);
  return buffer;
}
}
