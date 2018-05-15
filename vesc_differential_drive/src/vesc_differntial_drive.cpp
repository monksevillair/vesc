/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_differential_drive/vesc_differntial_drive.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>

namespace vesc_differntial_drive
{
VescDifferntialDrive::VescDifferntialDrive(ros::NodeHandle nh, ros::NodeHandle private_nh,
                                           const ros::NodeHandle &left_motor_private_nh,
                                           const ros::NodeHandle &right_motor_private_nh)
: nh_(nh), left_motor_(left_motor_private_nh, boost::bind(&VescDifferntialDrive::leftMotorSpeed, this, _1, _2),
                       boost::bind(&VescDifferntialDrive::batteryVoltage, this, _1)),
  has_left_motor_speed_(false),
  right_motor_(right_motor_private_nh, boost::bind(&VescDifferntialDrive::rightMotorSpeed, this, _1, _2)),
  has_right_motor_speed_(false), linear_velocity_odom_(0.), angular_velocity_odom_(0.), x_odom_(0.), y_odom_(0.),
  yaw_odom_(0.)
{
  if (!private_nh.getParam("max_velocity_linear", max_velocity_linear_))
  {
    ROS_ERROR("can't get paramter max_velocity_linear");
    throw std::invalid_argument("can't get paramter");
  }
  if (max_velocity_linear_ < 0.)
  {
    ROS_ERROR("max_velocity_linear is negative");
    throw std::invalid_argument("wrong range of paramter");
  }

  if (!private_nh.getParam("max_velocity_angular", max_velocity_angular_))
  {
    ROS_ERROR("can't get paramter max_velocity_angular");
    throw std::invalid_argument("can't get paramter");
  }
  if (max_velocity_angular_ < 0.)
  {
    ROS_ERROR("max_velocity_angular is negative");
    throw std::invalid_argument("wrong range of paramter");
  }

  if (!private_nh.getParam("track_width", track_width_))
  {
    ROS_ERROR("can't get paramter track_width");
    throw std::invalid_argument("can't get paramter");
  }

  if (!private_nh.getParam("wheel_diameter", wheel_diameter_))
  {
    ROS_ERROR("can't get paramter wheel_diameter");
    throw std::invalid_argument("can't get paramter");
  }

  velocity_correction_left_ = private_nh.param<double>("velocity_correction_left", 1.0);
  velocity_correction_right_ = private_nh.param<double>("velocity_correction_right", 1.0);

  if (!private_nh.getParam("allowed_brake_rpms", allowed_brake_rpms_))
  {
    ROS_ERROR("can't get paramter allowed_brake_rpms");
    throw std::invalid_argument("can't get paramter");
  }
  if (allowed_brake_rpms_ < 0.)
  {
    ROS_ERROR("allowed_brake_rpms is negative");
    throw std::invalid_argument("wrong range of paramter");
  }

  if (!private_nh.getParam("brake_rpms", brake_rpms_))
  {
    ROS_ERROR("can't get paramter brake_rpms");
    throw std::invalid_argument("can't get paramter");
  }
  if (brake_rpms_ < 0.)
  {
    ROS_ERROR("brake_rpms is negative");
    throw std::invalid_argument("wrong range of paramter");
  }

  if (!private_nh.getParam("brake_current", brake_current_))
  {
    ROS_ERROR("can't get paramter brake_current");
    throw std::invalid_argument("can't get paramter");
  }
  if (brake_rpms_ < 0.)
  {
    ROS_ERROR("brake_current is negative");
    throw std::invalid_argument("wrong range of paramter");
  }

  publish_odom_ = private_nh.param<bool>("publish_odom", true);
  if (publish_odom_)
  {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }

  publish_tf_ = private_nh.param<bool>("publish_tf", true);

  publish_motor_speed_ = private_nh.param<bool>("publish_motor_speed", false);

  odom_frame_ = private_nh.param<std::string>("odom_frame", "/odom");
  base_frame_ = private_nh.param<std::string>("base_frame", "/base_link");

  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &VescDifferntialDrive::commandVelocityCB, this);

  double odometry_time_frequency = private_nh.param<double>("odometry_time_frequency", 10.);

  // create a 20Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0 / (odometry_time_frequency / 2.)), &VescDifferntialDrive::timerCB, this);

  battery_voltage_pub_ = nh_.advertise<std_msgs::Float32>("/battery_voltage", 1);

  if (publish_motor_speed_)
  {
    left_motor_speed_pub_ = nh_.advertise<std_msgs::Float32>("/left_motor_speed", 1);
    right_motor_speed_pub_ = nh_.advertise<std_msgs::Float32>("/right_motor_speed", 1);

    left_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/left_velocity", 1);
    right_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/right_velocity", 1);

    left_motor_speed_send_pub_ = nh_.advertise<std_msgs::Float32>("/left_motor_speed_send", 1);
    right_motor_speed_send_pub_ = nh_.advertise<std_msgs::Float32>("/right_motor_speed_send", 1);

    left_velocity_send_pub_ = nh_.advertise<std_msgs::Float32>("/left_velocity_send", 1);
    right_velocity_send_pub_ = nh_.advertise<std_msgs::Float32>("/right_velocity_send", 1);
  }
}

void VescDifferntialDrive::timerCB(const ros::TimerEvent& /*event*/)
{
  if(!left_motor_.executionCycle() || !right_motor_.executionCycle())
  {
    ROS_FATAL("driver encoutered fault in execution cycle");
    timer_.stop();
    ros::shutdown();
  }
}

void VescDifferntialDrive::leftMotorSpeed(const double& speed, const ros::Time &time)
{
  has_left_motor_speed_ = true;
  left_motor_speed_ = speed;
  updateOdometry(time);
}

void VescDifferntialDrive::rightMotorSpeed(const double& speed, const ros::Time &time)
{
  has_right_motor_speed_ = true;
  right_motor_speed_ = speed;
  updateOdometry(time);
}

void VescDifferntialDrive::batteryVoltage(const double& voltage)
{
  std_msgs::Float32 battery_voltage_msg;
  battery_voltage_msg.data = voltage;

  battery_voltage_pub_.publish(battery_voltage_msg);
}

void VescDifferntialDrive::commandVelocityCB(const geometry_msgs::Twist &cmd_vel)
{
  const double linear_velocity = ensurBounds(cmd_vel.linear.x, max_velocity_linear_);
  const double angular_velocity = ensurBounds(cmd_vel.angular.z, max_velocity_angular_);

  const double left_velocity = (linear_velocity - angular_velocity * track_width_ / 2.) * velocity_correction_left_;
  const double right_velocity = (linear_velocity + angular_velocity * track_width_ / 2.) * velocity_correction_right_;

  const double left_rpm = left_velocity / (M_PI * wheel_diameter_) * 60.;
  const double right_rpm = right_velocity / (M_PI * wheel_diameter_) * 60.;

  if (publish_motor_speed_)
  {
    publishLeftVelocitySend(left_velocity);
    publishRightVelocitySend(right_velocity);
  }

  if ((std::fabs(left_rpm) <= brake_rpms_) && (std::fabs(right_rpm) <= brake_rpms_) &&
    (std::fabs(left_motor_speed_) <= allowed_brake_rpms_) && (std::fabs(right_motor_speed_) <= allowed_brake_rpms_))
  {
    ROS_DEBUG_STREAM("brake due to left_rpm: " << left_rpm << " right_rpm: " << right_rpm 
                    << " left_motor_speed: " << left_motor_speed_ << " right_motor_speed: " << right_motor_speed_);
    left_motor_.brake(brake_current_);
    right_motor_.brake(brake_current_);
  }
  else
  {
    left_motor_.sendRpms(left_rpm);
    right_motor_.sendRpms(right_rpm);

    if (publish_motor_speed_)
    {
      publishLeftMotorSpeedSend(left_rpm);
      publishRightMotorSpeedSend(right_rpm);
    }
  }
}

void VescDifferntialDrive::updateOdometry(const ros::Time time)
{
  if (!has_right_motor_speed_ || !has_left_motor_speed_)
    return;

  ROS_DEBUG_STREAM("left_motor_speed: " << left_motor_speed_ << " right_motor_speed: " << right_motor_speed_);

  const double left_velocity = left_motor_speed_ * M_PI * wheel_diameter_ / 60. / velocity_correction_left_;
  const double right_velocity = right_motor_speed_ * M_PI * wheel_diameter_ / 60. / velocity_correction_right_;

  if (publish_motor_speed_)
  {
    publishLeftMotorSpeed();
    publishRightMotorSpeed();
    publishLeftVelocity(left_velocity);
    publishRightVelocity(right_velocity);
  }

  ROS_DEBUG_STREAM("left_velocity: " << left_velocity << " right_velocity: " << right_velocity);

  if (!std::isfinite(left_velocity) || !std::isfinite(right_velocity))
  {
    ROS_ERROR_STREAM("velocity of motors is not valid (NAN or INF)" << " left_velocity: " << left_velocity
                                                                    << " right_velocity: " << right_velocity);
    return;
  }

  linear_velocity_odom_ = (left_velocity + right_velocity) / 2.;
  angular_velocity_odom_ = (right_velocity - left_velocity) / track_width_;

  if (odom_update_time_.isValid())
  {
    double time_difference = (time - odom_update_time_).toSec();

    if (time_difference < 0.)
    {
      ROS_WARN("time difference for odom update is negative skiping update");
      return;
    }

    x_odom_ += linear_velocity_odom_ * std::cos(yaw_odom_) * time_difference;
    y_odom_ += linear_velocity_odom_ * std::sin(yaw_odom_) * time_difference;
    yaw_odom_ += angular_velocity_odom_ * time_difference;
    yaw_odom_ = angles::normalize_angle(yaw_odom_);
  }

  odom_update_time_ = time;

  publishOdom();
}

void VescDifferntialDrive::publishOdom()
{
  const tf::Pose odom_pose(tf::createQuaternionFromYaw(yaw_odom_), tf::Vector3(x_odom_, y_odom_, 0.));

  if (publish_odom_)
  {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = odom_update_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    tf::poseTFToMsg(odom_pose, odom_msg.pose.pose);

    odom_msg.twist.twist.linear.x = linear_velocity_odom_;
    odom_msg.twist.twist.angular.z = angular_velocity_odom_;

    odom_pub_.publish(odom_msg);
  }

  if (publish_tf_)
  {
    tf::StampedTransform stamp_transform(odom_pose, odom_update_time_, odom_frame_, base_frame_);
    tf_broadcaster_.sendTransform(stamp_transform);
  }
}

double VescDifferntialDrive::ensurBounds(double value, double max)
{
  return ensurBounds(value, -max, max);
}

double VescDifferntialDrive::ensurBounds(double value, double min, double max)
{
  return std::min(std::max(value, min), max);
}

void VescDifferntialDrive::publishLeftMotorSpeed()
{
  publishDoubleValue(left_motor_speed_, left_motor_speed_pub_);
}

void VescDifferntialDrive::publishRightMotorSpeed()
{
  publishDoubleValue(right_motor_speed_, right_motor_speed_pub_);
}

void VescDifferntialDrive::publishLeftVelocity(const double &speed)
{
  publishDoubleValue(speed, left_velocity_pub_);
}

void VescDifferntialDrive::publishRightVelocity(const double &speed)
{
  publishDoubleValue(speed, right_velocity_pub_);
}

void VescDifferntialDrive::publishLeftMotorSpeedSend(const double &speed)
{
  publishDoubleValue(speed, left_motor_speed_send_pub_);
}

void VescDifferntialDrive::publishRightMotorSpeedSend(const double &speed)
{
  publishDoubleValue(speed, right_motor_speed_send_pub_);
}

void VescDifferntialDrive::publishLeftVelocitySend(const double &speed)
{
  publishDoubleValue(speed, left_velocity_send_pub_);
}

void VescDifferntialDrive::publishRightVelocitySend(const double &speed)
{
  publishDoubleValue(speed, right_velocity_send_pub_);
}

void VescDifferntialDrive::publishDoubleValue(const double& speed, ros::Publisher &motor_speed_pub)
{
  std_msgs::Float32 msg;
  msg.data = speed;
  motor_speed_pub.publish(msg);
}

}
