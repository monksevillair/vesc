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
  explicit VescMotor(const ros::NodeHandle& private_nh);

  /**
   * Gets the current motor velocity in rad/s, estimated at the given time.
   * @param time time at which velocity is estimated.
   * @return the estimated current velocity in rad/s.
   */
  double getVelocity(const ros::Time& time);

  /**
   * Commands the motor to revolute with the given velocity in rad/s.
   * @param velocity the velocity command in rad/s.
   */
  void setVelocity(double velocity);

  /**
   * Commands the motor to brake until it has stopped moving.
   * @param current the current (in A) to apply to the motor.
   */
  void brake(double current);

  /**
   * Gets the motor controller's supply voltage in V.
   * @return the supply voltage in V.
   */
  double getSupplyVoltage();

  /**
   * Requests an information update from the motor controller. Initializes the motor controller if it is not already
   * initialized.
   * @return true on success, false on error (e.g., if the motor controller has been disconnected).
   */
  bool executionCycle();

private:
  void reconfigure(MotorConfig& config, uint32_t level);
  void servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& servo_sensor_value);
  void stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state);
  double getVelocityConversionFactor() const;

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
  double supply_voltage_;
};
}

#endif // VESC_DIFFERENTIAL_DRIVE_VESC_MOTOR_H
