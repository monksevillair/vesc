/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/vesc_ros_driver.h>

namespace vesc_driver
{
VescRosDriver::VescRosDriver(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : driver_(nh, private_nh,
            boost::bind(&VescRosDriver::servoSensorCB, this, _1),
            boost::bind(&VescRosDriver::stateCB, this, _1))
{
  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>("sensors/core", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ = nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = nh.subscribe("commands/motor/duty_cycle", 10,
                                 &VescRosDriver::dutyCycleCallback, this);
  current_sub_ = nh.subscribe("commands/motor/current", 10, &VescRosDriver::currentCallback, this);
  brake_sub_ = nh.subscribe("commands/motor/brake", 10, &VescRosDriver::brakeCallback, this);
  speed_sub_ = nh.subscribe("commands/motor/speed", 10, &VescRosDriver::speedCallback, this);
  position_sub_ = nh.subscribe("commands/motor/position", 10, &VescRosDriver::positionCallback, this);
  servo_sub_ = nh.subscribe("commands/servo/position", 10, &VescRosDriver::servoCallback, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0 / 50.0), &VescRosDriver::timerCallback, this);
}

void VescRosDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  driver_.setDutyCycle(duty_cycle);
}

void VescRosDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  driver_.setCurrent(current);
}

void VescRosDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  driver_.setBrake(brake);
}

void VescRosDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  driver_.setSpeed(speed);
}

void VescRosDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  driver_.setPosition(position);
}

void VescRosDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  driver_.setServo(servo);
}

void VescRosDriver::timerCallback(const ros::TimerEvent& /*event*/)
{
  if(!driver_.executionCycle())
  {
    ROS_FATAL("driver encoutered fault in execution cycle");
    timer_.stop();
    ros::shutdown();
  }
}

void VescRosDriver::servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& servo_sensor_value)
{
  servo_sensor_pub_.publish(servo_sensor_value);
}

void VescRosDriver::stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state)
{
  state_pub_.publish(state);
}

}
