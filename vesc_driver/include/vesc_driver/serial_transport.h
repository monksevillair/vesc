/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_SERIAL_TRANSPORT_H
#define VESC_DRIVER_SERIAL_TRANSPORT_H

#include <atomic>
#include <boost/crc.hpp>
#include <cstdint>
#include <map>
#include <mutex>
#include <serial/serial.h>
#include <thread>
#include <vector>
#include <vesc_driver/blocking_queue.h>
#include <vesc_driver/serial_packet_codec.h>
#include <vesc_driver/transport.h>

namespace vesc_driver
{
class SerialTransport : public Transport
{
public:
  SerialTransport(uint8_t controller_id, const std::string& port);
  ~SerialTransport() override;

  void submit(TransportRequest&& r) override;

  void registerPacketHandler(uint8_t controller_id, PacketHandler&& packet_handler) override;

  void connect();

  bool isConnected();

  void disconnect();

protected:
  typedef std::vector<uint8_t> Buffer;
  typedef std::map<uint8_t, Transport::PacketHandler> PacketHandlers;
  typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> CRC;

  constexpr static size_t MIN_FRAME_SIZE = 5;
  constexpr static size_t MIN_LARGE_FRAME_SIZE = 6;
  constexpr static size_t SMALL_FRAME_SOF_BYTE = 2;
  constexpr static size_t LARGE_FRAME_SOF_BYTE = 3;
  constexpr static size_t EOF_BYTE = 3;

  void writeLoop();

  void write(const Buffer& buffer);

  void readLoop();

  void stopThreads();

  uint8_t controller_id_;

  std::mutex should_respond_mutex_;
  bool should_respond_ = false;
  uint8_t respond_id_ = 0;
  std::condition_variable should_respond_condition_;

  serial::Serial serial_port_;

  std::atomic<bool> should_write_{false};
  BlockingQueue<TransportRequest> write_queue_;
  std::thread write_thread_;

  std::atomic<bool> should_read_{false};
  std::thread read_thread_;
  std::mutex packet_handler_mutex_;
  PacketHandlers packet_handlers_;

  SerialPacketCodec packet_codec_;

  uint8_t readStartByte(Buffer& buffer);

  void readBytes(size_t size, Buffer& buffer);

  void readBytesUntilSizeReached(size_t size, Buffer& buffer);

  Buffer encodePacket(const TransportRequest& request);
  Buffer addFrame(const Buffer& payload);
};
}

#endif //VESC_DRIVER_SERIAL_TRANSPORT_H
