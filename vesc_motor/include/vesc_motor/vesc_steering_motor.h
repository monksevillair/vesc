/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_MOTOR_VESC_STEERING_MOTOR_H
#define VESC_MOTOR_VESC_STEERING_MOTOR_H

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vesc_motor/SteeringMotorConfig.h>
#include <vesc_motor/vesc_motor.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_motor
{
class VescSteeringMotor : public VescMotor
{
public:
  VescSteeringMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory,
                    double execution_duration);

  /**
   * Gets the current position motor of the motor in rad, estimated at the given time.
   * @param time time at which velocity is estimated.
   * @return the estimated position in rad.
   */
  double getPosition(const ros::Time& time);

  /**
   * Commands the motor to the position in rad.
   * @param position the position command in rad.
   */
  void setPosition(double position);

private:
  void reconfigure(SteeringMotorConfig& config, uint32_t level);
  void processMotorControllerState(const vesc_driver::MotorControllerState& state) override;

  bool predict(const ros::Time& time);
  void correct(double position);

  std::mutex config_mutex_;
  dynamic_reconfigure::Server<SteeringMotorConfig> reconfigure_server_;
  SteeringMotorConfig config_;

  std::mutex state_mutex_;
  cv::KalmanFilter position_kf_;
  ros::Time last_prediction_time_;
};
}

#endif //VESC_MOTOR_VESC_STEERING_MOTOR_H
