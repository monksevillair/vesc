/*
Created by clemens on 6/14/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_driver/vesc_driver_impl.h>
#include <functional>
#include <ros/console.h>
#include <vesc_driver/transport.h>

namespace vesc_driver
{
VescDriverImpl::VescDriverImpl(const std::chrono::duration<double>& sleep_duration,
                               const StateHandlerFunction& state_handler_function, uint8_t controller_id,
                               TransportPtr transport)
  : VescDriverInterface(state_handler_function), task_(std::bind(&VescDriverImpl::execute, this), sleep_duration),
    transport_(std::move(transport)), controller_id_(controller_id)
{
  task_.start();
  transport_->registerPacketHandler(
    controller_id, std::bind(&VescDriverImpl::processResponsePacket, this, std::placeholders::_1));
  transport_->registerTimeoutHandler(controller_id, std::bind(&VescDriverImpl::processTransportTimeout, this));
}

void VescDriverImpl::setDutyCycle(double duty_cycle)
{
  SetDutyCyclePacket packet;
  packet.duty_cycle = duty_cycle;
  transport_->submit(TransportRequest(controller_id_, std::move(packet)));
}

void VescDriverImpl::setCurrent(double current)
{
  SetCurrentPacket packet;
  packet.current = current;
  transport_->submit(TransportRequest(controller_id_, std::move(packet)));
}

void VescDriverImpl::setBrake(double brake)
{
  SetBrakePacket packet;
  packet.brake_current = brake;
  transport_->submit(TransportRequest(controller_id_, std::move(packet)));
}

void VescDriverImpl::setSpeed(double speed)
{
  ROS_DEBUG_STREAM("VescDriverImpl::setSpeed::1;" << controller_id_ << " speed: " << speed);
  SetSpeedPacket packet;
  packet.speed = speed;
  transport_->submit(TransportRequest(controller_id_, std::move(packet)));
}

void VescDriverImpl::setPosition(double position)
{
  SetPositionPacket packet;
  packet.position = position;
  transport_->submit(TransportRequest(controller_id_, std::move(packet)));
}

FirmwareVersion VescDriverImpl::getFirmwareVersion()
{
  return firmware_version_;
}

void VescDriverImpl::execute()
{
  ROS_DEBUG_STREAM("VescDriverImpl::execute::1;" << static_cast<int>(controller_id_));

  std::lock_guard<std::mutex> wait_for_response_lock(wait_for_response_mutex_);
  if (wait_for_response_)
  {
    return;
  }

  if (!initialized_)
  {
    ROS_DEBUG_STREAM("VescDriverImpl::execute::2;" << static_cast<int>(controller_id_));

    transport_->submit(TransportRequest(controller_id_, GetFirmwareVersionPacket(), true));
    wait_for_response_ = true;

    ROS_DEBUG_STREAM("VescDriverImpl::execute::3;" << static_cast<int>(controller_id_));
  }
  else
  {
    ROS_DEBUG_STREAM("VescDriverImpl::execute::4;" << static_cast<int>(controller_id_));

    transport_->submit(TransportRequest(controller_id_, GetValuesPacket(), true));
    wait_for_response_ = true;

    ROS_DEBUG_STREAM("VescDriverImpl::execute::5;" << static_cast<int>(controller_id_));
  }
}

void VescDriverImpl::processResponsePacket(const ResponsePacket& packet)
{
  std::unique_lock<std::mutex> wait_for_response_lock(wait_for_response_mutex_);
  if (wait_for_response_)
  {
    wait_for_response_ = false;
    wait_for_response_lock.unlock();

    boost::apply_visitor(ResponsePacketVisitor(this), packet);
  }
  else
  {
    wait_for_response_lock.unlock();

    ROS_WARN_STREAM("VescDriverImpl::processResponsePacket: received response packet without waiting for it");
  }
}

void VescDriverImpl::processTransportTimeout()
{
  std::unique_lock<std::mutex> wait_for_response_lock(wait_for_response_mutex_);
  wait_for_response_ = false;
}

void VescDriverImpl::processFirmwareVersionPacket(const FirmwareVersion& firmware_version)
{
  ROS_INFO_STREAM("VescDriverImpl::processFirmwareVersionPacket::1;" << static_cast<int>(controller_id_));
  firmware_version_ = firmware_version;
  initialized_ = true;
  ROS_INFO_STREAM("VescDriverImpl::processFirmwareVersionPacket::2;" << static_cast<int>(controller_id_));
}

void VescDriverImpl::processMotorControllerStatePacket(const MotorControllerState& state)
{
  state_handler_function_(state);
}

VescDriverImpl::ResponsePacketVisitor::ResponsePacketVisitor(VescDriverImpl* driver)
  : driver_(driver)
{
}

void VescDriverImpl::ResponsePacketVisitor::operator()(const MotorControllerState& packet) const
{
  driver_->processMotorControllerStatePacket(packet);
}

void VescDriverImpl::ResponsePacketVisitor::operator()(const FirmwareVersion& packet) const
{
  driver_->processFirmwareVersionPacket(packet);
}
}
