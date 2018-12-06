/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_motor/vesc_steering_motor.h>
#include <functional>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_motor
{

VescSteeringMotor::VescSteeringMotor(const ros::NodeHandle& private_nh, const DriverFactoryPtr& driver_factory,
                                     const std::chrono::duration<double>& execution_duration)
  : VescMotor(private_nh, driver_factory, execution_duration), reconfigure_server_(private_nh)
{
  // Init Kalman filter:
  unsigned int state_size = 2; // [p, v]
  unsigned int meas_size = 1; // [p]
  unsigned int contr_size = 0; // []

  position_kf_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [p, v]
  position_kf_.statePost.at<float>(0) = 0; // p
  position_kf_.statePost.at<float>(1) = 0; // v

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(position_kf_.transitionMatrix);

  // Measurement matrix (H):
  // [ 1 0]
  position_kf_.measurementMatrix.at<float>(0) = 1.0f;
  position_kf_.measurementMatrix.at<float>(1) = 0.0f;

  // Process noise covariance matrix (Q):
  // [ Ep 0  ]
  // [ 0  Ev ]
  position_kf_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  position_kf_.processNoiseCov.at<float>(1, 1) = 1.0f;

  // Measurement noise covariance matrix (R):
  cv::setIdentity(position_kf_.measurementNoiseCov, 1e-1);

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(position_kf_.errorCovPre);

  reconfigure_server_.setCallback(std::bind(&VescSteeringMotor::reconfigure, this, std::placeholders::_1));
}

double VescSteeringMotor::getPosition(const ros::Time& time)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return position_kf_.statePre.at<float>(0);
}

double VescSteeringMotor::getVelocity(const ros::Time& time)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return position_kf_.statePre.at<float>(1);
}

void VescSteeringMotor::setPosition(double position)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  const double position_in_rad = position * (config_.invert_direction ? -1. : 1.) + config_.position_offset;
  const double position_in_deg = position_in_rad / M_PI * 180.0;
  ROS_DEBUG_STREAM("VescSteeringMotor::setPosition: this: " << this << ", position_in_deg: " << position_in_deg);
  driver_->setPosition(position_in_deg);
}

void VescSteeringMotor::reconfigure(SteeringMotorConfig& config)
{
  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  std::unique_lock<std::mutex> config_lock(config_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::1");

  config_ = config;

  if (!driver_)
  {
    createDriver();
  }

  if (driver_->isMockup())
  {
    std::shared_ptr<vesc_driver::VescDriverMockup> casted_driver = std::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_);

    if (!casted_driver)
    {
      ROS_ERROR("Mockup can not be casted to mokup class");
    }
    else
    {
      casted_driver->setMaxCurrent(config_.mockup_max_current);
      casted_driver->setCurrentToAcceleration(config_.mockup_current_to_acceleration);
    }
  }

  // Process noise covariance matrix (Q):
  // [ Ep 0  ]
  // [ 0  Ev ]
  position_kf_.processNoiseCov.at<float>(0, 0) = config_.process_noise_p;
  position_kf_.processNoiseCov.at<float>(1, 1) = config_.process_noise_v;

  // Measurement noise covariance matrix (R):
  cv::setIdentity(position_kf_.measurementNoiseCov, config_.measurement_noise);

  ROS_DEBUG_STREAM("VescSteeringMotor::reconfigure::2");
}

void VescSteeringMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
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
    const double position_in_deg = state.position;
    ROS_DEBUG_STREAM("VescSteeringMotor::processMotorControllerState: this: " << this << ", position_in_deg: "
                                                                             << position_in_deg);
    const double position_in_rad = position_in_deg / 180. * M_PI;
    const double position = (position_in_rad - config_.position_offset) / (config_.invert_direction ? -1. : 1.);
    correct(position);
  }
  else
  {
    if (!driver_->isMockup())
      ROS_WARN("Skipping state correction due to failed prediction");
  }
//  ROS_INFO_STREAM("VescSteeringMotor::processMotorControllerState: position after correction: "
//                    << position_kf_.statePost.at<float>(0));
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
