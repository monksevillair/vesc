/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_MOCKUP_H
#define VESC_DRIVER_VESC_DRIVER_MOCKUP_H

#include <mutex>
#include <ros/time.h>
#include <vesc_driver/packet.h>
#include <vesc_driver/periodic_task.h>
#include <vesc_driver/vesc_driver_interface.h>

namespace vesc_driver
{
class VescDriverMockup : public VescDriverInterface
{
public:
  VescDriverMockup(const std::chrono::duration<double>& sleep_duration,
                   const StateHandlerFunction& state_handler_function);

  void setDutyCycle(double duty_cycle) override;

  void setCurrent(double current) override;

  void setBrake(double brake) override;

  void setSpeed(double speed) override;

  void setPosition(double position) override;

  FirmwareVersion getFirmwareVersion() override;

  bool isMockup() override;

protected:
  enum class ControlMode
  {
    DUTY_CYCLE, CURRENT, BRAKE, SPEED, POSITION
  };

  void execute();
  void updateState();

  PeriodicTask task_;
  std::mutex state_mutex_;
  ControlMode control_mode_ = ControlMode::SPEED;
  double command_ = 0.0;
  double last_position_ = 0.0;
  ros::Time last_update_time_;
  MotorControllerState state_;
};
}

#endif //VESC_DRIVER_VESC_DRIVER_MOCKUP_H
