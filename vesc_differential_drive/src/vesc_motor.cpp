/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_differential_drive/vesc_motor.h>
#include <boost/bind.hpp>
#include <limits>
#include <stdexcept>
#include <vesc_driver/vesc_driver_serial.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_differential_drive
{
VescMotor::VescMotor(const ros::NodeHandle& private_nh, double execution_duration)
  : private_nh_(private_nh), reconfigure_server_(private_nh), execution_duration_(execution_duration),
    supply_voltage_(std::numeric_limits<double>::quiet_NaN())
{
  reconfigure_server_.setCallback(boost::bind(&VescMotor::reconfigure, this, _1, _2));

  // Init Kalman filter:
  unsigned int state_size = 2; // [v, a]
  unsigned int meas_size = 1; // [v]
  unsigned int contr_size = 0; // []

  speed_kf_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [v, a]
  speed_kf_.statePost.at<float>(0) = 0; // v
  speed_kf_.statePost.at<float>(1) = 0; // a

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(speed_kf_.transitionMatrix);

  // Measurement matrix (H):
  // [ 1 0]
  speed_kf_.measurementMatrix.at<float>(0) = 1.0f;
  speed_kf_.measurementMatrix.at<float>(1) = 0.0f;

  // Process noise covariance matrix (Q):
  // [ Ev 0  ]
  // [ 0  Ea ]
  speed_kf_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  speed_kf_.processNoiseCov.at<float>(1, 1) = 1.0f;

  // Measurement noise covariance matrix (R):
  cv::setIdentity(speed_kf_.measurementNoiseCov, 1e-1);

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(speed_kf_.errorCovPre);
}

double VescMotor::getVelocity(const ros::Time& time)
{
  boost::mutex::scoped_lock state_lock(state_mutex_);
  predict(time);
  return speed_kf_.statePre.at<float>(0);
}

void VescMotor::setVelocity(double velocity)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  driver_->setSpeed(velocity);
}

void VescMotor::brake(double current)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  driver_->setBrake(current);
}

double VescMotor::getSupplyVoltage()
{
  boost::mutex::scoped_lock state_lock(state_mutex_);
  return supply_voltage_;
}

void VescMotor::reconfigure(MotorConfig& config, uint32_t /*level*/)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  boost::mutex::scoped_lock state_lock(state_mutex_);

  config_ = config;

  if (config_.motor_poles == 0.0)
  {
    ROS_ERROR("Parameter motor_poles is not set");
  }

  if (config_.use_mockup)
  {
    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_))
    {
      driver_.reset(new vesc_driver::VescDriverMockup(execution_duration_, boost::bind(&VescMotor::stateCB, this, _1)));
    }
  }
  else
  {
    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverSerial>(driver_))
    {
      int controller_id;
      private_nh_.getParam("controller_id", controller_id);
      std::string port;
      private_nh_.getParam("port", port);
      driver_.reset(new vesc_driver::VescDriverSerial(execution_duration_, boost::bind(&VescMotor::stateCB, this, _1),
                                                      controller_id, port));
    }
  }
}

void VescMotor::stateCB(const vesc_driver::MotorControllerState& state)
{
  boost::mutex::scoped_lock state_lock(state_mutex_);

  ros::Time now = ros::Time::now();
  if (predict(now)) // only correct if prediction can be performed
  {
    correct(state.speed / getVelocityConversionFactor());
  }
  else
  {
    ROS_WARN("Skipping state correction due to failed prediction");
  }

  supply_voltage_ = state.voltage_input;
}

double VescMotor::getVelocityConversionFactor() const
{
  // The Vesc controller expects and reports velocities in electrical RPM:
  return (config_.invert_direction ? -1.0 : 1.0) * config_.motor_poles * config_.velocity_correction * 30.0 * M_1_PI;
}

bool VescMotor::predict(const ros::Time& time)
{
  if (time > last_prediction_time_)
  {
    if (!last_prediction_time_.isZero())
    {
      const double dt = (time - last_prediction_time_).toSec();
      speed_kf_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      speed_kf_.predict();
    }

    last_prediction_time_ = time;
    return true;
  }
  return false;
}

void VescMotor::correct(double velocity)
{
  // Kalman Correction
  const cv::Vec<float, 1> measurement(static_cast<float>(velocity));
  speed_kf_.correct(cv::Mat(measurement, false));
}

}
