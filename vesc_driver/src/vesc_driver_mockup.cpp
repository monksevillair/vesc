/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_driver
{
VescDriverMockup::VescDriverMockup(const std::chrono::duration<double>& sleep_duration,
                                   const VescDriverInterface::StateHandlerFunction& state_handler_function)
  :
  VescDriverInterface(state_handler_function), PeriodicTask(sleep_duration)
{}

void VescDriverMockup::setDutyCycle(double duty_cycle)
{
  std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
  current_state_.duty_cycle = duty_cycle;
}

void VescDriverMockup::setCurrent(double current)
{
  std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
  current_state_.current_motor = current;
}

void VescDriverMockup::setBrake(double brake)
{
  std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
  current_state_.current_motor = brake;
}

void VescDriverMockup::setSpeed(double speed)
{
  std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
  current_state_.speed = speed;
}

void VescDriverMockup::setPosition(double position)
{
  std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
  current_state_.position = position;
}

FirmwareVersion VescDriverMockup::getFirmwareVersion()
{
  FirmwareVersion firmware_version;
  firmware_version.major_version = 3;
  firmware_version.minor_version = 2;
  return firmware_version;
}

void VescDriverMockup::execute()
{
  MotorControllerState state_to_send;

  {
    std::lock_guard<std::mutex> current_state_lock(current_state_mutex_);
    state_to_send = current_state_;
  }

  state_handler_function_(state_to_send);
}

}
