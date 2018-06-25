/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_motor/vesc_drive_motor.h>
#include <boost/bind.hpp>
#include <limits>
#include <stdexcept>
#include <vesc_driver/vesc_driver_serial.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_motor
{
VescDriveMotor::VescDriveMotor(const ros::NodeHandle& private_nh, std::shared_ptr<VescTransportFactory> transport_factory, double execution_duration)
  : private_nh_(private_nh), reconfigure_server_(private_nh_), transport_factory_(transport_factory), execution_duration_(execution_duration),
    supply_voltage_(std::numeric_limits<double>::quiet_NaN())
{
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::1");

  // Init Kalman filter:
  unsigned int state_size = 2; // [v, a]
  unsigned int meas_size = 1; // [v]
  unsigned int contr_size = 0; // []
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::2");

  speed_kf_ = cv::KalmanFilter(state_size, meas_size, contr_size, CV_32F);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::3");

  // Corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)):
  // [v, a]
  speed_kf_.statePost.at<float>(0) = 0; // v
  speed_kf_.statePost.at<float>(1) = 0; // a
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::4");

  // State transition matrix (A):
  // [ 1 dT]
  // [ 0 1]
  // Note: set dT at each processing step!
  cv::setIdentity(speed_kf_.transitionMatrix);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::5");

  // Measurement matrix (H):
  // [ 1 0]
  speed_kf_.measurementMatrix.at<float>(0) = 1.0f;
  speed_kf_.measurementMatrix.at<float>(1) = 0.0f;
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::6");

  // Process noise covariance matrix (Q):
  // [ Ev 0  ]
  // [ 0  Ea ]
  speed_kf_.processNoiseCov.at<float>(0, 0) = 1e-2f;
  speed_kf_.processNoiseCov.at<float>(1, 1) = 1.0f;
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::7");

  // Measurement noise covariance matrix (R):
  cv::setIdentity(speed_kf_.measurementNoiseCov, 1e-1);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::8");

  // Priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q):
  cv::setIdentity(speed_kf_.errorCovPre);
  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::9");

  reconfigure_server_.setCallback(boost::bind(&VescDriveMotor::reconfigure, this, _1, _2));

  ROS_DEBUG_STREAM("VescDriveMotor::VescDriveMotor::10");
}

double VescDriveMotor::getVelocity(const ros::Time& time)
{
  boost::mutex::scoped_lock state_lock(state_mutex_);
  predict(time);
  return speed_kf_.statePre.at<float>(0);
}

void VescDriveMotor::setVelocity(double velocity)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  driver_->setSpeed(velocity * getVelocityConversionFactor());
}

void VescDriveMotor::brake(double current)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  driver_->setBrake(current);
}

double VescDriveMotor::getSupplyVoltage()
{
  boost::mutex::scoped_lock state_lock(state_mutex_);
  return supply_voltage_;
}

void VescDriveMotor::reconfigure(DriveMotorConfig& config, uint32_t /*level*/)
{
  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  boost::mutex::scoped_lock state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::1");

  config_ = config;

  ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::2");

  if (config_.motor_poles == 0.0)
  {
    ROS_ERROR("Parameter motor_poles is not set");
  }

  if (config_.use_mockup)
  {
    ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::3");

    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverMockup>(driver_))
    {
      ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::4");

      driver_.reset(new vesc_driver::VescDriverMockup(execution_duration_, boost::bind(&VescDriveMotor::stateCB, this, _1)));
    }
  }
  else
  {
    ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::5");

    if (!boost::dynamic_pointer_cast<vesc_driver::VescDriverSerial>(driver_))
    {
      ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::6");

      int controller_id;
      private_nh_.getParam("controller_id", controller_id);

      ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::7");
      ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::7.1 controller_id: " << controller_id);

      if (private_nh_.hasParam("port"))
      {
        ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::8");

        std::string port;
        private_nh_.getParam("port", port);

        ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::8.1 port: " << port);

        driver_.reset(new vesc_driver::VescDriverSerial(execution_duration_, boost::bind(&VescDriveMotor::stateCB, this, _1),
                                                        controller_id, port));
      }
      else
      {
        ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::9");

        std::string transport_name;
        private_nh_.getParam("transport_name", transport_name);

        std::shared_ptr<vesc_driver::SerialTransport> transport = transport_factory_->getSerialTransport(transport_name);
        driver_.reset(new vesc_driver::VescDriverSerial(execution_duration_, boost::bind(&VescDriveMotor::stateCB, this, _1),
                                                        controller_id, transport));
      }

      ROS_DEBUG_STREAM("VescDriveMotor::reconfigure::10");
    }
  }
}

void VescDriveMotor::stateCB(const vesc_driver::MotorControllerState& state)
{
  ROS_DEBUG_STREAM("VescDriveMotor::stateCB::1");

  boost::mutex::scoped_lock state_lock(state_mutex_);

  ROS_DEBUG_STREAM("VescDriveMotor::stateCB::2");

  ros::Time now = ros::Time::now();
  if (predict(now)) // only correct if prediction can be performed
  {
    ROS_DEBUG_STREAM("VescDriveMotor::stateCB::3");

    correct(state.speed / getVelocityConversionFactor());
  }
  else
  {
    ROS_WARN("Skipping state correction due to failed prediction");
  }

  ROS_DEBUG_STREAM("VescDriveMotor::stateCB::4 state.voltage_input: " << state.voltage_input);

  supply_voltage_ = state.voltage_input;
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
      speed_kf_.transitionMatrix.at<float>(0, 1) = static_cast<float>(dt);
      speed_kf_.predict();
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
  speed_kf_.correct(cv::Mat(measurement, false));

  ROS_DEBUG_STREAM("VescDriveMotor::correct::2");
}

}
