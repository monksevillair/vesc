/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DIFFERENTIAL_DRIVE_VESC_MOTOR_H
#define VESC_DIFFERENTIAL_DRIVE_VESC_MOTOR_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vesc_differential_drive/MotorConfig.h>
#include <vesc_driver/vesc_driver_interface.h>

namespace vesc_differential_drive
{
class VescMotor
{
public:
  explicit VescMotor(ros::NodeHandle private_nh);

  void setVelocity(double rpm);

  void brake(double current);

  bool executionCycle();

  double getVoltage();

  double getVelocity(const ros::Time& time);

private:
  void reconfigure(MotorConfig& config, uint32_t level);
  void servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& servo_sensor_value);
  void stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state);

  bool predict(const ros::Time &time);
  void correct(double velocity);

  ros::NodeHandle private_nh_;
  dynamic_reconfigure::Server<MotorConfig> reconfigure_server_;
  MotorConfig config_;

  boost::mutex driver_mutex_;
  boost::shared_ptr<vesc_driver::VescDriverInterface> driver_;

  boost::mutex state_mutex_;

  cv::KalmanFilter speed_kf_;
  ros::Time last_prediction_time_;

  double current_voltage_;
};
}

#endif // VESC_DIFFERENTIAL_DRIVE_VESC_MOTOR_H
