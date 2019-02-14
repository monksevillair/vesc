/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/vesc_driver_mockup.h>
#include <cmath>
#include <functional>

namespace vesc_driver
{
VescDriverMockup::VescDriverMockup(const std::chrono::duration<double>& sleep_duration,
                                   const VescDriverInterface::StateHandlerFunction& state_handler_function)
  : VescDriverInterface(state_handler_function), task_(std::bind(&VescDriverMockup::execute, this), sleep_duration),
    max_current_(10.), current_to_acceleration_(90.)
{
  task_.start();
}

void VescDriverMockup::setDutyCycle(double duty_cycle)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  updateState();
  control_mode_ = ControlMode::DUTY_CYCLE;
  command_ = duty_cycle;
}

void VescDriverMockup::setCurrent(double current)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  updateState();
  control_mode_ = ControlMode::CURRENT;
  command_ = current;
}

void VescDriverMockup::setBrake(double brake)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  updateState();
  control_mode_ = ControlMode::BRAKE;
  command_ = brake;
}

void VescDriverMockup::setSpeed(double speed)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  updateState();
  control_mode_ = ControlMode::SPEED;
  command_ = speed;
}

void VescDriverMockup::setPosition(double position)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  updateState();
  control_mode_ = ControlMode::POSITION;
  command_ = position;
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
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    updateState();
    state_to_send = state_;
  }

  state_handler_function_(state_to_send);
}

bool VescDriverMockup::isMockup()
{
  return true;
}

void VescDriverMockup::updateState()
{
  const ros::Time now = ros::Time::now();
  if (!last_update_time_.isZero() && now > last_update_time_)
  {
    const double dt = std::min((now - last_update_time_).toSec(), 1.0);
    double current = 0.0;
    switch (control_mode_)
    {
      case ControlMode::DUTY_CYCLE:
      {
        current = command_ * max_current_;
        break;
      }

      case ControlMode::CURRENT:
      {
        current = command_;
        break;
      }

      case ControlMode::BRAKE:
      {
        if (std::fabs(state_.speed) > 1.0)
        {
          current = (state_.speed < 0.0) ? command_ : -command_;
        }
        break;
      }

      case ControlMode::SPEED:
      {
        const double error = command_ - state_.speed;
        if (std::fabs(error) > 1.0)
        {
          current = (error >= 0.0 ? max_current_ : -max_current_);
        }
        break;
      }

      case ControlMode::POSITION:
      {
        const double error = command_ - state_.position;
        const double last_error = command_ - last_position_;
        const double error_rate = (error - last_error) / dt;
        current = std::min(std::max(-1.0, error * 0.1 + error_rate * 0.05), 1.0) * max_current_;
        break;
      }
    }

    state_.current_motor = current;
    state_.duty_cycle = current / max_current_;
    state_.speed += current * current_to_acceleration_ * dt;
    last_position_ = state_.position;
    state_.position += state_.speed * dt;
  }
  last_update_time_ = now;
}

void VescDriverMockup::setMaxCurrent(double max_current)
{
  max_current_ = max_current;
}

void VescDriverMockup::setCurrentToAcceleration(double current_to_acceleration)
{
  current_to_acceleration_ = current_to_acceleration;
}

void VescDriverMockup::setStatePosition(double pos)
{
  state_.position = pos * 180.0 / M_PI;
}

}
