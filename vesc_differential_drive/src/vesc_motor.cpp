/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_differential_drive/vesc_motor.h>

namespace vesc_differntial_drive
{
VescMotor::VescMotor(ros::NodeHandle private_nh,
                     const SpeedHandlerFunction &speed_handler_function)
: send_rpms_(false), write_thread_(boost::bind(&VescMotor::run, this)),
  speed_handler_function_(speed_handler_function), driver_(private_nh,
                                                           boost::bind(&VescMotor::servoSensorCB, this, _1),
                                                           boost::bind(&VescMotor::stateCB, this, _1))
{
  if (!private_nh.getParam("motor_pols", motor_pols_))
    throw std::invalid_argument("motor pols are not defined");

  invert_direction_ = private_nh.param<bool>("invert_direction", false);
}

void VescMotor::sendRpms(double rpm)
{
  boost::mutex::scoped_lock write_lock(write_buffer_mutex_);
  buffered_rpms_ = rpm;
  send_rpms_ = true;
  write_buffer_condition_.notify_all();
}

void VescMotor::servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& /*servo_sensor_value*/)
{ }

void VescMotor::stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state)
{
  speed_handler_function_(state->state.speed / motor_pols_ * (invert_direction_ ? -1. : 1.), state->header.stamp);
}

void VescMotor::run()
{
  while(ros::ok())
  {
    boost::mutex::scoped_lock write_lock(write_buffer_mutex_);
    while (!send_rpms_)
      write_buffer_condition_.wait(write_lock);

    ROS_INFO_STREAM("buffered_rpms: " << buffered_rpms_);

    std_msgs::Float64::Ptr motor_speed(new std_msgs::Float64());
    motor_speed->data = buffered_rpms_ * motor_pols_ * (invert_direction_ ? -1. : 1.);
    driver_.setSpeed(motor_speed);
    send_rpms_ = false;
  }
}

bool VescMotor::executionCycle()
{
  return driver_.executionCycle();
}
}
