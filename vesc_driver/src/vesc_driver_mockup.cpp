/*
Created by clemens on 5/16/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_driver
{
  VescDriverMockup::VescDriverMockup(const StateHandlerFunction &state_handler) :
      VescDriverInterface(ServoSensorHandlerFunction(), state_handler), send_state_(false),
      state_thread_(boost::bind(&VescDriverMockup::run, this))
  { }

  void VescDriverMockup::setDutyCycle(const std_msgs::Float64::ConstPtr &duty_cycle)
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);
    current_state_.state.duty_cycle = duty_cycle->data;
  }

  void VescDriverMockup::setCurrent(const std_msgs::Float64::ConstPtr &current)
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);
    current_state_.state.current_motor = current->data;
  }

  void VescDriverMockup::setBrake(const std_msgs::Float64::ConstPtr &brake)
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);
    current_state_.state.current_motor = brake->data;
  }

  void VescDriverMockup::setSpeed(const std_msgs::Float64::ConstPtr &speed)
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);
    current_state_.state.speed = speed->data;
  }

  void VescDriverMockup::setPosition(const std_msgs::Float64::ConstPtr &)
  { }

  void VescDriverMockup::setServo(const std_msgs::Float64::ConstPtr &)
  { }

  bool VescDriverMockup::executionCycle()
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);
    send_state_ = true;
    send_state_condition_.notify_all();

    return true;
  }

  void VescDriverMockup::run()
  {
    boost::mutex::scoped_lock state_lock(state_mutex_);

    while (!send_state_)
      send_state_condition_.wait(state_lock);

    while (ros::ok())
    {
      while (!send_state_)
        send_state_condition_.wait(state_lock);

      vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
      state_msg->header.stamp = ros::Time::now();
      state_msg->state.current_motor = current_state_.state.current_motor;
      state_msg->state.speed = current_state_.state.speed;
      state_msg->state.duty_cycle = current_state_.state.duty_cycle;
      state_msg->state.fault_code = vesc_msgs::VescState::FAULT_CODE_NONE;

      state_handler_(state_msg);

      send_state_ = false;
    }
  }
}