/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_SERIAL_TRANSPORT_H
#define VESC_DRIVER_SERIAL_TRANSPORT_H

#include <vesc_driver/byte_buffer.h>
#include <vesc_driver/transport.h>
#include <serial/serial.h>
#include <vesc_driver/serial_packet_codec.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vesc_driver/blocking_queue.h>
#include <map>

namespace vesc_driver
{
  class SerialTransport : public Transport
  {
  public:
    explicit SerialTransport(uint8_t controller_id);

    void submit(TransportRequest &&r) override;

    void registerPacketHandler(uint8_t controller_id, PacketHandler &&pacekt_handler) override;

    void connect(const std::string &port);

    bool isConnected();

    void disconnect();

  protected:
    void writeLoop();

    void write(ByteBuffer buffer);

    void readLoop();

    void stopThreads();

    uint8_t controller_id_;

    std::mutex should_respond_mutex_;
    bool should_respond_;
    uint8_t respond_id_;
    std::condition_variable should_respond_condition_;

    serial::Serial serial_port_;

    std::atomic<bool> should_write_;
    BlockingQueue<TransportRequest> write_queue_;
    std::thread write_thread_;

    std::atomic<bool> should_read_;
    std::thread read_thread_;
    std::mutex pacekt_handler_mutex_;
    std::map<uint8_t, Transport::PacketHandler> packet_handlers_;

    SerialPacketCodec packet_codec_;

    void readStartByte(ByteBuffer &buffer);

    void readBytes(const size_t size, ByteBuffer &buffer);

    void readPayloadSize(size_t bytes, ByteBuffer &buffer);

    void readPayload(size_t size, uint16_t buffer, ByteBuffer &byteBuffer);

    bool performCRCCheck(const ByteBuffer &buffer, uint16_t expected_crc_value);

    ByteBuffer extractPayload(size_t bytes, uint16_t size, const ByteBuffer &buffer);
  };
}

#endif //VESC_DRIVER_SERIAL_TRANSPORT_H
