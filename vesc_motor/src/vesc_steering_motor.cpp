/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_motor/vesc_steering_motor.h>
#include <functional>

namespace vesc_motor
{

VescSteeringMotor::VescSteeringMotor(const ros::NodeHandle& private_nh,
                                     std::shared_ptr<VescTransportFactory> transport_factory, double execution_duration)
  : VescMotor(private_nh, transport_factory, execution_duration), reconfigure_server_(private_nh)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::1");

  // Init Kalman filter:
  unsigned int state_size = 2; // [p, v]
  unsigned int meas_size = 1; // [p]
  unsigned int contr_size = 0; // []
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::2");

  position_kf_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::3");

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [p, v]
  position_kf_.statePost.at<float>(0) = 0; // p
  position_kf_.statePost.at<float>(1) = 0; // v
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::4");

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(position_kf_.transitionMatrix);
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::5");

  // Measurement matrix (H):
  // [ 1 0]
  position_kf_.measurementMatrix.at<float>(0) = 1.0f;
  position_kf_.measurementMatrix.at<float>(1) = 0.0f;
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::6");

  // Process noise covariance matrix (Q):
  // [ Ev 0  ]
  // [ 0  Ea ]
  position_kf_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  position_kf_.processNoiseCov.at<float>(1, 1) = 1.0f;
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(position_kf_.measurementNoiseCov, 1e-1);
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::8");

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(position_kf_.errorCovPre);
  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::9");

  reconfigure_server_.setCallback(
    std::bind(&VescSteeringMotor::reconfigure, this, std::placeholders::_1, std::placeholders::_2));

  ROS_DEBUG_STREAM("VescSteeringMotor::VescSteeringMotor::10");
}

double VescSteeringMotor::getPosition(const ros::Time& time)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return position_kf_.statePre.at<float>(0);
}

void VescSteeringMotor::setPosition(double position)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setPosition(position * (config_.invert_direction ? -1. : 1.) + config_.position_offset);
}

void VescSteeringMotor::reconfigure(SteeringMotorConfig& config, uint32_t level)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  std::unique_lock<std::mutex> config_lock(config_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  config_ = config;

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::2");

  updateDriver(config_.use_mockup);
}

void VescSteeringMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);

//  ROS_INFO_STREAM("VescSteeringMotor::processMotorControllerState: position before prediction: "
//                    << position_kf_.statePost.at<float>(0));
  ros::Time now = ros::Time::now();
  if (predict(now)) // only correct if prediction can be performed
  {
//    ROS_INFO_STREAM("VescSteeringMotor::processMotorControllerState: position after prediction: "
//                      << position_kf_.statePre.at<float>(0));
//    ROS_INFO_STREAM("VescSteeringMotor::processMotorControllerState: position measurement: "
//                      << (state.position * (config_.invert_direction ? -1. : 1.) - config_.position_offset));
    correct(state.position * (config_.invert_direction ? -1. : 1.) - config_.position_offset);
  }
  else
  {
    ROS_WARN("Skipping state correction due to failed prediction");
  }
//  ROS_INFO_STREAM("VescSteeringMotor::processMotorControllerState: position after correction: "
//                    << position_kf_.statePost.at<float>(0));

  // Call super class implementation:
  VescMotor::processMotorControllerState(state);
}

bool VescSteeringMotor::predict(const ros::Time& time)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::predict::1");

  if (time > last_prediction_time_)
  {
    ROS_DEBUG_STREAM("VescSteeringMotor::predict::2");

    if (!last_prediction_time_.isZero())
    {
      ROS_DEBUG_STREAM("VescSteeringMotor::predict::3");

      const double dt = (time - last_prediction_time_).toSec();
      position_kf_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      position_kf_.predict();
    }

    ROS_DEBUG_STREAM("VescSteeringMotor::predict::4");

    last_prediction_time_ = time;
    return true;
  }

  ROS_DEBUG_STREAM("VescSteeringMotor::predict::5");

  return false;
}

void VescSteeringMotor::correct(double position)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::correct::1 position: " << position);

  // Kalman Correction
  const cv::Vec<float, 1> measurement(static_cast<float>(position));
  position_kf_.correct(cv::Mat(measurement, false));

  ROS_DEBUG_STREAM("VescSteeringMotor::correct::2");
}
}
