/*
Created by clemens on 6/14/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/vesc_driver_serial.h>

namespace vesc_driver
{
  VescDriverSerial::VescDriverSerial(const std::chrono::duration<double> &sleep_duration,
                                     const vesc_driver::VescDriverInterface::StateHandlerFunction &state_handler_function,
                                     uint8_t controller_id, const std::string &port) :
      VescDriverInterface(state_handler_function), PeriodicExecution(sleep_duration), serial_transport_(controller_id),
      packet_visitor_(std::bind(&VescDriverSerial::receiveFirmwareVersion, this, std::placeholders::_1),
                      std::bind(&VescDriverSerial::receiveMotorControllerStateCB, this, std::placeholders::_1)),
      initialized_(false), major_fw_version_(0), minor_fw_version_(0), controller_id_(controller_id)
  {
    serial_transport_.registerPacketHandler(controller_id, std::bind(&VescDriverSerial::processPacketVariant, this,
                                                                     std::placeholders::_1));

    serial_transport_.connect(port);
  }

  void VescDriverSerial::setDutyCycle(double duty_cycle)
  {
    SetDutyCyclePacket packet;
    packet.duty_cycle = duty_cycle;

    TransportRequest request(controller_id_, packet);

    serial_transport_.submit(std::forward<TransportRequest>(request));
  }

  void VescDriverSerial::setCurrent(double current)
  {
    SetCurrentPacket packet;
    packet.current = current;

    TransportRequest request(controller_id_, packet);

    serial_transport_.submit(std::forward<TransportRequest>(request));
  }

  void VescDriverSerial::setBrake(double brake)
  {
    SetBrakePacket packet;
    packet.brake_current = brake;

    TransportRequest request(controller_id_, packet);

    serial_transport_.submit(std::forward<TransportRequest>(request));
  }

  void VescDriverSerial::setSpeed(double speed)
  {
    SetSpeedPacket packet;
    packet.speed = speed;

    TransportRequest request(controller_id_, packet);

    serial_transport_.submit(std::forward<TransportRequest>(request));
  }

  void VescDriverSerial::setPosition(double position)
  {
    SetPositionPacket packet;
    packet.position = position;

    TransportRequest request(controller_id_, packet);

    serial_transport_.submit(std::forward<TransportRequest>(request));
  }

  void VescDriverSerial::execution()
  {
    if (!initialized_)
    {
      GetFirmwareVersion packet;

      TransportRequest request(controller_id_, packet);

      serial_transport_.submit(std::forward<TransportRequest>(request));
    }
    else
    {
      GetValuesPacket packet;

      TransportRequest request(controller_id_, packet);

      serial_transport_.submit(std::forward<TransportRequest>(request));
    }
  }

  void VescDriverSerial::receiveFirmwareVersion(const FirmwareVersion &fw_version)
  {
    major_fw_version_ = fw_version.major_version;
    minor_fw_version_ = fw_version.minor_version;

    initialized_ = true;
  }

  void VescDriverSerial::receiveMotorControllerStateCB(const MotorControllerState& state)
  {
    state_handler_function_(state);
  }

  VescDriverSerial::PacketVariantVisitor::PacketVariantVisitor(
      const VescDriverSerial::PacketVariantVisitor::ReceiveFirmwareVersionCB &receive_firmware_version_cb,
      const VescDriverSerial::PacketVariantVisitor::ReceiveMotorControllerStateCB &receive_motor_controller_state_cb) :
      receive_firmware_version_cb_(receive_firmware_version_cb),
      receive_motor_controller_state_cb_(receive_motor_controller_state_cb)
  { }

  void VescDriverSerial::PacketVariantVisitor::operator()(const SetDutyCyclePacket &packet)
  {
    throw std::runtime_error("received invalid SetDutyCyclePacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const SetCurrentPacket &packet)
  {
    throw std::runtime_error("received invalid SetCurrentPacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const SetBrakePacket &packet)
  {
    throw std::runtime_error("received invalid SetBrakePacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const SetSpeedPacket &packet)
  {
    throw std::runtime_error("received invalid SetSpeedPacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const SetPositionPacket &packet)
  {
    throw std::runtime_error("received invalid SetPositionPacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const GetValuesPacket &packet)
  {
    throw std::runtime_error("received invalid GetValuesPacket packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const MotorControllerState &packet)
  {
    receive_motor_controller_state_cb_(packet);
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const GetFirmwareVersion &packet)
  {
    throw std::runtime_error("received invalid GetFirmwareVersion packet");
  }

  void VescDriverSerial::PacketVariantVisitor::operator()(const FirmwareVersion &packet)
  {
    receive_firmware_version_cb_(packet);
  }
}
