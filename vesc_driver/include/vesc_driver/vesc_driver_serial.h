/*
Created by clemens on 6/14/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_SERIAL_H
#define VESC_DRIVER_VESC_DRIVER_SERIAL_H

#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_driver/periodic_execution.h>
#include <vesc_driver/serial_transport.h>

namespace vesc_driver
{
class VescDriverSerial : public VescDriverInterface, public PeriodicExecution
{
public:
  VescDriverSerial(const std::chrono::duration<double>& sleep_duration,
                   const StateHandlerFunction& state_handler_function, uint8_t controller_id, const std::string& port);

  VescDriverSerial(const std::chrono::duration<double>& sleep_duration,
                   const StateHandlerFunction& state_handler_function, uint8_t controller_id,
                   std::shared_ptr<SerialTransport> serial_transport);

  void setDutyCycle(double duty_cycle) override;

  void setCurrent(double current) override;

  void setBrake(double brake) override;

  void setSpeed(double speed) override;

  void setPosition(double position) override;

  std::shared_ptr<SerialTransport> getSerialTransport();

protected:
  class PacketVariantVisitor : public boost::static_visitor<>
  {
  public:
    typedef std::function<void(const FirmwareVersion&)> ReceiveFirmwareVersionCB;
    typedef std::function<void(const MotorControllerState&)> ReceiveMotorControllerStateCB;

    PacketVariantVisitor(const ReceiveFirmwareVersionCB& receive_firmware_version_cb,
                         const ReceiveMotorControllerStateCB& receive_motor_controller_state_cb);

    void operator()(const SetDutyCyclePacket& packet);
    void operator()(const SetCurrentPacket& packet);
    void operator()(const SetBrakePacket& packet);
    void operator()(const SetSpeedPacket& packet);
    void operator()(const SetPositionPacket& packet);
    void operator()(const GetValuesPacket& packet);
    void operator()(const MotorControllerState& packet);
    void operator()(const GetFirmwareVersion& packet);
    void operator()(const FirmwareVersion& packet);

  protected:
    ReceiveFirmwareVersionCB receive_firmware_version_cb_;
    ReceiveMotorControllerStateCB receive_motor_controller_state_cb_;
  };

  void execution() override;

  void processPacketVariant(const PacketVariant& packet)
  {
    boost::apply_visitor(packet_visitor_, packet);
  }

  void receiveFirmwareVersion(const FirmwareVersion& fw_version);
  void receiveMotorControllerStateCB(const MotorControllerState& state);

  std::shared_ptr<SerialTransport> serial_transport_;

  PacketVariantVisitor packet_visitor_;

  std::atomic<bool> initialized_;

  std::mutex wait_for_response_mutex_;
  bool wait_for_response_;

  uint8_t major_fw_version_;
  uint8_t minor_fw_version_;

  uint8_t controller_id_;
};
}

#endif //VESC_DRIVER_VESC_DRIVER_SERIAL_H
