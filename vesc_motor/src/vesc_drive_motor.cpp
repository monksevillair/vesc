/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_motor/vesc_drive_motor.h>
#include <vesc_driver/motor_controller_state.h>
#include <vesc_driver/vesc_driver_interface.h>
#include <functional>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_motor
{
VescDriveMotor::VescDriveMotor(
  const ros::NodeHandle& private_nh, const DriverFactoryPtr& driver_factory,
  const std::chrono::duration<double>& execution_duration)
  : VescMotor(private_nh, driver_factory, execution_duration), reconfigure_server_(private_nh)
{
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::1");

  // Init Kalman filter:
  unsigned int state_size = 2; // [v, a]
  unsigned int meas_size = 1; // [v]
  unsigned int contr_size = 0; // []
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::2");

  state_estimation_filter_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::3");

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [v, a]
  state_estimation_filter_.statePost.at<float>(0) = 0; // v
  state_estimation_filter_.statePost.at<float>(1) = 0; // a
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::4");

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(state_estimation_filter_.transitionMatrix);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::5");

  // Measurement matrix (H):
  // [ 1 0]
  state_estimation_filter_.measurementMatrix.at<float>(0) = 1.0f;
  state_estimation_filter_.measurementMatrix.at<float>(1) = 0.0f;
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::6");

  // Process noise covariance matrix (Q):
  // [ Ev 0  ]
  // [ 0  Ea ]
  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = 1.0f;
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, 1e-1);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::8");

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(state_estimation_filter_.errorCovPre);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::9");

  reconfigure_server_.setCallback(std::bind(&VescDriveMotor::reconfigure, this, std::placeholders::_1));

  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::10");
}

double VescDriveMotor::getVelocity(const ros::Time& time)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  predict(time);
  return state_estimation_filter_.statePre.at<float>(0);
}

void VescDriveMotor::setVelocity(double velocity)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setSpeed(velocity * getVelocityConversionFactor());
  //if (private_nh_.getNamespace().rfind("/front_axle/left_motor") != std::string::npos)
  //{
  //  ROS_INFO_STREAM("VescDriveMotor::setVelocity: " << private_nh_.getNamespace() << " -- velocity: " << velocity);
  //}
}

void VescDriveMotor::brake(double current)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  driver_->setBrake(current);
}

void VescDriveMotor::reconfigure(DriveMotorConfig& config)
{
  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  std::unique_lock<std::mutex> config_lock(config_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  config_ = config;

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::2");

  if (config_.motor_poles == 0.0)
  {
    ROS_ERROR("Parameter motor_poles is not set");
  }

  if (!driver_)
  {
    createDriver();
  }

  if (driver_->isMockup())
  {
    const auto casted_driver = std::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_);

    if (!casted_driver)
    {
      ROS_ERROR("Mockup can not be casted to mockup class");
    }
    else
    {
      casted_driver->setMaxCurrent(config_.mockup_max_current);
      casted_driver->setCurrentToAcceleration(config_.mockup_current_to_acceleration);
    }
  }

  // Process noise covariance matrix (Q):
  // [ Ev 0  ]
  // [ 0  Ea ]
  state_estimation_filter_.processNoiseCov.at<float>(0, 0) = config_.process_noise_v;
  state_estimation_filter_.processNoiseCov.at<float>(1, 1) = config_.process_noise_a;
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(state_estimation_filter_.measurementNoiseCov, config_.measurement_noise);
}

void VescDriveMotor::processMotorControllerState(const vesc_driver::MotorControllerState& state)
{
  std::unique_lock<std::mutex> config_lock(config_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescDriveMotor::processMotorControllerState: speed: " << state.speed);

  ros::Time now = ros::Time::now();
  if (predict(now)) // only correct if prediction can be performed
  {
    ROS_DEBUG_STREAM("VescDriveMotor::processMotorControllerState::3");

    correct(state.speed / getVelocityConversionFactor());
  }
  else if (!driver_->isMockup())
  {
    ROS_WARN("Skipping state correction due to failed prediction");
  }

  ROS_DEBUG_STREAM("VescDriveMotor::processMotorControllerState: corrected velocity: "
                     << state_estimation_filter_.statePost.at<float>(0));
}

double VescDriveMotor::getVelocityConversionFactor() const
{
  // The Vesc controller expects and reports velocities in electrical RPM:
  return (config_.invert_direction ? -1.0 : 1.0) * config_.motor_poles * config_.velocity_correction * 30.0 * M_1_PI;
}

bool VescDriveMotor::predict(const ros::Time& time)
{
  ROS_DEBUG_STREAM("VescDriveMotor::predict::1");

  if (time > last_prediction_time_)
  {
    ROS_DEBUG_STREAM("VescDriveMotor::predict::2");

    if (!last_prediction_time_.isZero())
    {
      ROS_DEBUG_STREAM("VescDriveMotor::predict::3");

      const double dt = (time - last_prediction_time_).toSec();
      state_estimation_filter_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      state_estimation_filter_.predict();

      //if (private_nh_.getNamespace().rfind("/front_axle/left_motor") != std::string::npos)
      //{
      //  ROS_INFO_STREAM("VescDriveMotor::predict: " << private_nh_.getNamespace() << " -- v: "
      //                                              << state_estimation_filter_.statePre.at<float>(0) << ", a: "
      //                                              << state_estimation_filter_.statePre.at<float>(1) << ", dt: " << dt);
      //}
    }

    ROS_DEBUG_STREAM("VescDriveMotor::predict::4");

    last_prediction_time_ = time;
    return true;
  }

  ROS_DEBUG_STREAM("VescDriveMotor::predict::5");

  return false;
}

void VescDriveMotor::correct(double velocity)
{
  ROS_DEBUG_STREAM("VescDriveMotor::correct::1 velocity: " << velocity);

  // Kalman Correction
  const cv::Vec<float, 1> measurement(static_cast<float>(velocity));
  state_estimation_filter_.correct(cv::Mat(measurement, false));

  //if (private_nh_.getNamespace().rfind("/front_axle/left_motor") != std::string::npos)
  //{
  //  ROS_INFO_STREAM("VescDriveMotor::correct: " << private_nh_.getNamespace() << " -- v: "
  //                                              << state_estimation_filter_.statePost.at<float>(0) << ", a: "
  //                                              << state_estimation_filter_.statePost.at<float>(1) << ", velocity: " << velocity);
  //}
  ROS_DEBUG_STREAM("VescDriveMotor::correct::2");
}
}
