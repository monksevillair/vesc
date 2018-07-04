/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef vesc_motor_VESC_MOTOR_H
#define vesc_motor_VESC_MOTOR_H

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vesc_motor/DriveMotorConfig.h>
#include <vesc_motor/vesc_motor.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_motor
{
class VescDriveMotor : public VescMotor
{
public:
  VescDriveMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory,
                 double execution_duration);

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

protected:
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  void reconfigure(DriveMotorConfig& config, uint32_t level);
  double getVelocityConversionFactor() const;

  bool predict(const ros::Time &time);
  void correct(double velocity);

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<DriveMotorConfig> reconfigure_server_;
  DriveMotorConfig config_;

  std::mutex state_mutex_;
  cv::KalmanFilter speed_kf_;
  ros::Time last_prediction_time_;
};
}

#endif // vesc_motor_VESC_MOTOR_H
