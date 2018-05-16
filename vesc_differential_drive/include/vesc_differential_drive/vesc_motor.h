/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DIFFERENTIAL_DRIVE_VESCMOTOR_H
#define VESC_DIFFERENTIAL_DRIVE_VESCMOTOR_H

#include <vesc_driver/vesc_driver_interface.h>
#include <boost/thread.hpp>
#include <ros/ros.h>

namespace vesc_differntial_drive
{
class VescMotor
{
public:
  typedef boost::function<void (const double&, const ros::Time& time)> SpeedHandlerFunction;
  typedef boost::function<void (const double&)> VoltageHandlerFunction;

  VescMotor(ros::NodeHandle private_nh, const SpeedHandlerFunction& speed_handler_function,
            const VoltageHandlerFunction& voltage_handler_function = VoltageHandlerFunction());

  void sendRpms(double rpm);

  void brake(double current);

  void servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& servo_sensor_value);

  void stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state);

  bool executionCycle();

private:
  double motor_pols_;

  bool invert_direction_;

  boost::mutex write_buffer_mutex_;
  boost::condition_variable write_buffer_condition_;
  bool send_rpms_;
  double buffered_rpms_;
  boost::thread write_thread_;

  bool send_brake_;
  double buffered_brake_current_;

  SpeedHandlerFunction speed_handler_function_;
  VoltageHandlerFunction voltage_handler_function_;

  vesc_driver::VescDriverInterface* driver_;

  void run();
};
}

#endif //VESC_DIFFERENTIAL_DRIVE_VESCMOTOR_H
