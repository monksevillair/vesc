/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/serial_transport.h>
#include <stdexcept>

namespace vesc_driver
{

  SerialTransport::SerialTransport(uint8_t controller_id) : controller_id_(controller_id), should_respond_(false),
                                                            serial_port_(std::string(), 115200,
                                                                         serial::Timeout::simpleTimeout(100),
                                                                         serial::eightbits, serial::parity_none,
                                                                         serial::stopbits_one, serial::flowcontrol_none),
                                                            write_thread_(&SerialTransport::writeLoop, this)
  { }

  void SerialTransport::submit(TransportRequest &&r)
  {
    write_queue_.push(std::forward<TransportRequest>(r));
  }

  void SerialTransport::connect(const std::string &port)
  {
    if (isConnected())
      throw std::logic_error("Try to connect to " + port + " when a connection already exists");

    stopThreads();

    try
    {
      serial_port_.setPort(port);
      serial_port_.open();
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("Encountered error during opening to port " + port + " error: " + e.what());
    }

    should_read_ = true;
    read_thread_ = std::thread(&SerialTransport::readLoop, this);
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

  void SerialTransport::write(ByteBuffer buffer)
  {
    if (isConnected())
    {
      size_t bytes_written = serial_port_.write(buffer);

      if (bytes_written != buffer.getSize())
      {
        throw std::runtime_error("Can not write the complete buffer");
      }
    }
    else
    {
      throw std::runtime_error("Try to write without previously opened port");
    }
  }

  void SerialTransport::readLoop()
  {
    while(should_read_)
    {
      ByteBuffer read_buffer;
      readStartByte(read_buffer);
      if (!should_read_)
        break;

      uint8_t start_byte = read_buffer.parsUnsignedInt8();

      size_t payload_size_bytes;
      if (start_byte == SerialPacketCodec::VESC_SMALL_FRAME)
        payload_size_bytes = 1;
      else if (start_byte == SerialPacketCodec::VESC_LARGE_FRAME)
        payload_size_bytes = 2;
      else
      {
        //skipping invalid input
        continue;
      }
      readPayloadSize(payload_size_bytes, read_buffer);
      if (!should_read_)
        break;

      uint16_t payload_size;
      if (start_byte == SerialPacketCodec::VESC_SMALL_FRAME)
        payload_size = read_buffer.parsUnsignedInt8();
      else if (start_byte == SerialPacketCodec::VESC_LARGE_FRAME)
        payload_size = read_buffer.parsUnsignedInt16();

      readPayload(payload_size_bytes, payload_size, read_buffer);
      if (!should_read_)
        break;

      read_buffer.reverseAdvanceTo(0);
      uint8_t eof_byte = read_buffer.parsUnsignedInt8();
      if (eof_byte != SerialPacketCodec::VESC_EOF_BYTE)
      {
        //skipping invalid input
        continue;
      }

      read_buffer.reverseAdvanceTo(2);
      uint16_t crc_value = read_buffer.parsUnsignedInt16();

      ByteBuffer payload = extractPayload(payload_size_bytes, payload_size, read_buffer);
      if(!performCRCCheck(payload, crc_value))
      {
        //skipping invalid input
        continue;
      }

      boost::optional<PacketVariant> paket = packet_codec_.decode(payload);

      if (!paket)
      {
        //skipping invalid input
        continue;
      }

      {
        std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);
        if (should_respond_)
        {
          should_respond_ = false;

          std::lock_guard<std::mutex> pacekt_handler_lock(pacekt_handler_mutex_);
          if (packet_handlers_.find(respond_id_) == packet_handlers_.end())
          {
            // no packet handler registered
            continue;
          }
          else
            packet_handlers_[respond_id_](paket.get());
        }
      }
    }
  }

  void SerialTransport::registerPacketHandler(uint8_t controller_id, Transport::PacketHandler &&pacekt_handler)
  {
    std::lock_guard<std::mutex> pacekt_handler_lock(pacekt_handler_mutex_);
    packet_handlers_[controller_id] = pacekt_handler;
  }

  void SerialTransport::writeLoop()
  {
    while(should_write_)
    {
      try
      {
        TransportRequest request = write_queue_.pop();

        {
          std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);
          while (should_respond_)
            should_respond_condition_.wait(should_respond_lock);
        }

        ByteBuffer write_buffer;

        if (request.getControllerId() == controller_id_)
          packet_codec_.encode(request.getPacket(), write_buffer);
        else
          packet_codec_.fowardCan(request.getPacket(), write_buffer, request.getControllerId());

        {
          std::unique_lock<std::mutex> should_respond_lock(should_respond_mutex_);

          write(write_buffer);

          if (request.expectResponse())
          {
            should_respond_ = true;
            respond_id_ = request.getControllerId();
          }
        }
      }
      catch (const InterruptException& exception)
      {
        break;
      }
    }
  }

  void SerialTransport::stopThreads()
  {
    bool should_read_expected_value = true;
    if (should_write_.compare_exchange_strong(should_read_expected_value, false) && write_thread_.joinable())
    {
      write_queue_.interrupt();
      write_thread_.join();
    }

    bool should_write_expected_value = true;
    if (should_read_.compare_exchange_strong(should_write_expected_value, false) && read_thread_.joinable())
      read_thread_.join();
  }

  void SerialTransport::readStartByte(ByteBuffer &buffer)
  {
    while (should_read_)
    {
      buffer.clear();

      readBytes(SerialPacketCodec::VESC_MIN_FRAME_SIZE, buffer);

      while (buffer.canFurtherPars())
      {
        uint8_t start_byte = buffer.parsUnsignedInt8();

        if ((start_byte == SerialPacketCodec::VESC_SMALL_FRAME) || (start_byte == SerialPacketCodec::VESC_LARGE_FRAME))
        {
          buffer.reverseStep(1);
          buffer.reduceToParsingPoint();
          return;
        }
      }
    }
  }

  void SerialTransport::readBytes(const size_t size, ByteBuffer &buffer)
  {
    size_t number_bytes_to_read = std::max(size, serial_port_.available());

    size_t read_bytes = 0;
    while (should_read_ && (read_bytes < size))
    {
      size_t remaining_bytes = number_bytes_to_read - read_bytes;
      std::vector<uint8_t> bytes_to_read(remaining_bytes);

      size_t new_read_bytes = serial_port_.read(bytes_to_read, remaining_bytes);

      if (new_read_bytes > 0)
        buffer.addBytes(bytes_to_read);

      read_bytes += new_read_bytes;
    }
  }

  void SerialTransport::readPayloadSize(size_t bytes, ByteBuffer &buffer)
  {
    size_t needed_bytes = 1 + bytes; // at lest start byte and payload needs to be read

    if (needed_bytes > buffer.getSize())
    {
      readBytes(needed_bytes - buffer.getSize(), buffer);
    }
  }

  void SerialTransport::readPayload(size_t payload_size_bytes, uint16_t payload_size, ByteBuffer buffer)
  {
    // at lest start byte, payload size, payload, crc, eof byte needs to be read
    size_t needed_bytes = 1 + payload_size_bytes + payload_size + 2 + 1;

    if (needed_bytes > buffer.getSize())
    {
      readBytes(needed_bytes - buffer.getSize(), buffer);
    }
  }

  bool SerialTransport::performCRCCheck(const ByteBuffer &buffer, uint16_t expected_crc_value)
  {
    std::vector<uint8_t> bytes = buffer;
    SerialPacketCodec::CRC crc_calculation;
    crc_calculation.process_bytes(&(*bytes.begin()), std::distance(bytes.begin(), bytes.end()));
    return expected_crc_value == crc_calculation.checksum();
  }

  ByteBuffer SerialTransport::extractPayload(size_t payload_size_bytes, uint16_t payload_size,
                                             const ByteBuffer &buffer)
  {
    std::vector<uint8_t> bytes = buffer;
    bytes.erase(bytes.begin(), bytes.begin() + 1 + payload_size_bytes); // remove header
    bytes.erase(bytes.begin() + payload_size, bytes.begin() + payload_size + 2 + 1); // remove footer

    return ByteBuffer(std::forward<std::vector<uint8_t>>(bytes));
  }
}
