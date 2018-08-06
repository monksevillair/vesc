/*
Created by clemens on 6/14/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_IMPL_H
#define VESC_DRIVER_VESC_DRIVER_IMPL_H

#include <atomic>
#include <boost/variant/static_visitor.hpp>
#include <vesc_driver/packet.h>
#include <vesc_driver/periodic_task.h>
#include <vesc_driver/types.h>
#include <vesc_driver/vesc_driver_interface.h>

namespace vesc_driver
{
class VescDriverImpl : public VescDriverInterface
{
public:
  VescDriverImpl(const std::chrono::duration<double>& sleep_duration,
                 const StateHandlerFunction& state_handler_function, uint8_t controller_id, TransportPtr transport);

  void setDutyCycle(double duty_cycle) override;

  void setCurrent(double current) override;

  void setBrake(double brake) override;

  void setSpeed(double speed) override;

  void setPosition(double position) override;

  FirmwareVersion getFirmwareVersion() override;

protected:
  struct ResponsePacketVisitor : public boost::static_visitor<>
  {
    explicit ResponsePacketVisitor(VescDriverImpl* driver);

    void operator()(const MotorControllerState& packet) const;
    void operator()(const FirmwareVersion& packet) const;

    VescDriverImpl* driver_;
  };

  void execute();

  void processResponsePacket(const ResponsePacket& packet);
  void processTransportTimeout();
  void processFirmwareVersionPacket(const FirmwareVersion& firmware_version);
  void processMotorControllerStatePacket(const MotorControllerState& state);

  PeriodicTask task_;
  TransportPtr transport_;

  std::atomic<bool> initialized_{false};

  std::mutex wait_for_response_mutex_;
  bool wait_for_response_ = false;

  FirmwareVersion firmware_version_;

  uint8_t controller_id_;
};
}

#endif //VESC_DRIVER_VESC_DRIVER_IMPL_H
