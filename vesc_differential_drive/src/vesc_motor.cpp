/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_differential_drive/vesc_motor.h>
#include <vesc_driver/vesc_driver.h>
#include <vesc_driver/vesc_driver_mockup.h>

namespace vesc_differntial_drive
{
VescMotor::VescMotor(ros::NodeHandle private_nh)
: driver_(NULL), current_voltage_(-1.)
{
  if (!private_nh.getParam("motor_pols", motor_pols_))
    throw std::invalid_argument("motor pols are not defined");

  invert_direction_ = private_nh.param<bool>("invert_direction", false);

  bool use_mockup = private_nh.param<bool>("use_mockup", false);

  if (!use_mockup)
    driver_ = new vesc_driver::VescDriver(private_nh,
                                          boost::bind(&VescMotor::servoSensorCB, this, _1),
                                          boost::bind(&VescMotor::stateCB, this, _1));
  else
    driver_ = new vesc_driver::VescDriverMockup(boost::bind(&VescMotor::stateCB, this, _1));

  // init kalman filter
  // [v, a]
  unsigned int state_size = 2;
  // [v]
  unsigned int meas_size = 1;
  unsigned int contr_size = 0;
  unsigned short type = CV_32F;

  speed_kf_ = cv::KalmanFilter(state_size, meas_size, contr_size, type);
  speed_kf_state_ = cv::Mat(state_size, 1, type);
  speed_kf_measurement_ = cv::Mat(meas_size, 1, type);

  // Transition State Matrix A
  // Note: set dT at each processing step!
  // [ 1 dT]
  // [ 0 1]
  cv::setIdentity(speed_kf_.transitionMatrix);
  speed_kf_.transitionMatrix.at<float>(1) = 0.0;

  // Measure Matrix H
  // [ 1 0]
  speed_kf_.measurementMatrix = cv::Mat::zeros(meas_size, state_size, type);
  speed_kf_.measurementMatrix.at<float>(0) = 1.0f; //0 7 16 23

  // Process Noise Covariance Matrix Q
  // [ Ev 0  ]
  // [ 0  Ea ]
  speed_kf_.processNoiseCov.at<float>(0) = 1e-2;
  speed_kf_.processNoiseCov.at<float>(3) = 1.0f;

  // Measures Noise Covariance Matrix R
  cv::setIdentity(speed_kf_.measurementNoiseCov, cv::Scalar(1e-1));

  speed_kf_.errorCovPre.at<float>(0) = 1.0;
  speed_kf_.errorCovPre.at<float>(3) = 1.0;

  speed_kf_measurement_.at<float>(0) = 0.0;

  speed_kf_state_.at<float>(0) = speed_kf_measurement_.at<float>(0);
  speed_kf_state_.at<float>(1) = 0;
  speed_kf_state_.at<float>(2) = 0;
  speed_kf_state_.at<float>(3) = 0;

  speed_kf_.statePost = speed_kf_state_;
}

void VescMotor::sendRpms(double rpm)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  std_msgs::Float64::Ptr motor_speed(new std_msgs::Float64());
  motor_speed->data = rpm * motor_pols_ * (invert_direction_ ? -1. : 1.);
  driver_->setSpeed(motor_speed);}

void VescMotor::brake(double current)
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  std_msgs::Float64::Ptr brake_current(new std_msgs::Float64());
  brake_current->data = current;
  driver_->setBrake(brake_current);
}

void VescMotor::servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& /*servo_sensor_value*/)
{ }

void VescMotor::stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state)
{
  boost::mutex::scoped_lock state_lock(state_mutex_);

  if (predict(state->header.stamp)) // only correct if prediction can be performed
  {
    correct(state->state.speed / motor_pols_ * (invert_direction_ ? -1. : 1.));
  }
  else
  {
    ROS_WARN("skipping state cb due to negative time");
  }

  current_voltage_ = state->state.voltage_input;
}

bool VescMotor::executionCycle()
{
  boost::mutex::scoped_lock driver_lock(driver_mutex_);
  return driver_->executionCycle();
}

double VescMotor::getVoltage()
{
  boost::mutex::scoped_lock state_lock(state_mutex_);
  return current_voltage_;
}

double VescMotor::getSpeed(const ros::Time &time)
{
  boost::mutex::scoped_lock state_lock(state_mutex_);

  predict(time);

  return speed_kf_state_.at<float>(0);
}

bool VescMotor::predict(const ros::Time &time)
{
  double dt = 0.;

  if (last_predication_.isValid())
    dt = (time - last_predication_).toSec();

  bool predicted = false;
  if (dt > 0.)
  {
    speed_kf_.transitionMatrix.at<float>(1) = (float)dt;

    // let the kalman magic happen :P
    speed_kf_state_ = speed_kf_.predict();

    last_predication_ = time;

    predicted = true;
  }

  return predicted;
}

void VescMotor::correct(double speed)
{
  speed_kf_measurement_.at<float>(0) = static_cast<float>(speed);

  speed_kf_.correct(speed_kf_measurement_); // Kalman Correction
}


}
