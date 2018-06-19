/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_differential_drive/vesc_differential_drive.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <vesc_differential_drive/vesc_transport_factory.h>

namespace vesc_differential_drive
{
VescDifferentialDrive::VescDifferentialDrive(ros::NodeHandle private_nh, const ros::NodeHandle& left_motor_private_nh,
                                             const ros::NodeHandle& right_motor_private_nh)
  : initialized_(false), private_nh_(private_nh), reconfigure_server_(private_nh_),
    left_motor_(left_motor_private_nh, 1.0 / (config_.odometry_rate * 2.1)), left_motor_velocity_(0.),
    right_motor_(right_motor_private_nh, 1.0 / (config_.odometry_rate * 2.1)), right_motor_velocity_(0.),
    linear_velocity_odom_(0.), angular_velocity_odom_(0.), x_odom_(0.), y_odom_(0.), yaw_odom_(0.)
{
  if (private_nh_.hasParam("transport_mapping"))
  {
    std::shared_ptr<VescTransportFactory> transport_factory = std::make_shared<VescTransportFactory>(private_nh_);
    left_motor_.setTransportFactory(transport_factory);
    right_motor_.setTransportFactory(transport_factory);
  }

  reconfigure_server_.setCallback(boost::bind(&VescDifferentialDrive::reconfigure, this, _1, _2));

  if (config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }

  cmd_vel_sub_ = private_nh_.subscribe("/cmd_vel", 1, &VescDifferentialDrive::commandVelocityCB, this);

  odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate),
                                        &VescDifferentialDrive::odomTimerCB, this);

  battery_voltage_pub_ = private_nh_.advertise<std_msgs::Float32>("/battery_voltage", 1);

  if (config_.publish_motor_speed)
  {
    left_motor_speed_pub_ = private_nh_.advertise<std_msgs::Float32>("/left_motor_speed", 1);
    right_motor_speed_pub_ = private_nh_.advertise<std_msgs::Float32>("/right_motor_speed", 1);

    left_velocity_pub_ = private_nh_.advertise<std_msgs::Float32>("/left_velocity", 1);
    right_velocity_pub_ = private_nh_.advertise<std_msgs::Float32>("/right_velocity", 1);

    left_motor_speed_send_pub_ = private_nh_.advertise<std_msgs::Float32>("/left_motor_speed_send", 1);
    right_motor_speed_send_pub_ = private_nh_.advertise<std_msgs::Float32>("/right_motor_speed_send", 1);

    left_velocity_send_pub_ = private_nh_.advertise<std_msgs::Float32>("/left_velocity_send", 1);
    right_velocity_send_pub_ = private_nh_.advertise<std_msgs::Float32>("/right_velocity_send", 1);
  }

  initialized_ = true;
}

void VescDifferentialDrive::reconfigure(DifferentialDriveConfig& config, uint32_t /*level*/)
{
  if (config.max_velocity_linear == 0.0)
  {
    ROS_ERROR("Parameter max_velocity_linear is not set");
  }

  if (config.max_velocity_angular == 0.0)
  {
    ROS_ERROR("Parameter max_velocity_angular is not set");
  }

  if (config.track_width == 0.0)
  {
    ROS_ERROR("Parameter track_width is not set");
  }

  if (config.wheel_diameter == 0.0)
  {
    ROS_ERROR("Parameter wheel_diameter is not set");
  }

  if (config.allowed_brake_velocity == 0.0)
  {
    ROS_ERROR("Parameter allowed_brake_velocity is not set");
  }

  if (config.brake_velocity == 0.0)
  {
    ROS_ERROR("Parameter brake_velocity is not set");
  }

  if (config.brake_current == 0.0)
  {
    ROS_ERROR("Parameter brake_current is not set");
  }

  config_ = config;
}

void VescDifferentialDrive::odomTimerCB(const ros::TimerEvent& event)
{
  if (initialized_)
  {
    left_motor_velocity_ = left_motor_.getVelocity(event.current_real);
    right_motor_velocity_ = right_motor_.getVelocity(event.current_real);
    updateOdometry(event.current_real);
    publishDoubleValue(left_motor_.getSupplyVoltage(), battery_voltage_pub_);
  }
}

void VescDifferentialDrive::commandVelocityCB(const geometry_msgs::Twist& cmd_vel)
{
  const double linear_velocity = ensureBounds(cmd_vel.linear.x, config_.max_velocity_linear);
  const double angular_velocity = ensureBounds(cmd_vel.angular.z, config_.max_velocity_angular);

  const double left_tangential_velocity = (linear_velocity - angular_velocity * 0.5 * config_.track_width);
  const double right_tangential_velocity = (linear_velocity + angular_velocity * 0.5 * config_.track_width);

  if (config_.publish_motor_speed)
  {
    publishDoubleValue(left_tangential_velocity, left_velocity_send_pub_);
    publishDoubleValue(right_tangential_velocity, right_velocity_send_pub_);
  }

  const double left_rotational_velocity = left_tangential_velocity / (0.5 * config_.wheel_diameter);
  const double right_rotational_velocity = right_tangential_velocity / (0.5 * config_.wheel_diameter);

  if ((std::fabs(left_rotational_velocity) <= config_.brake_velocity)
    && (std::fabs(right_rotational_velocity) <= config_.brake_velocity)
    && (std::fabs(left_motor_velocity_) <= config_.allowed_brake_velocity)
    && (std::fabs(right_motor_velocity_) <= config_.allowed_brake_velocity))
  {
    ROS_DEBUG_STREAM("brake due to"
                       << " left_rotational_velocity: " << left_rotational_velocity
                       << " right_rotational_velocity: " << right_rotational_velocity
                       << " left_motor_velocity: " << left_motor_velocity_
                       << " right_motor_velocity: " << right_motor_velocity_);
    left_motor_.brake(config_.brake_current);
    right_motor_.brake(config_.brake_current);
  }
  else
  {
    left_motor_.setVelocity(left_rotational_velocity);
    right_motor_.setVelocity(right_rotational_velocity);

    if (config_.publish_motor_speed)
    {
      publishDoubleValue(left_rotational_velocity, left_motor_speed_send_pub_);
      publishDoubleValue(right_rotational_velocity, right_motor_speed_send_pub_);
    }
  }
}

void VescDifferentialDrive::updateOdometry(const ros::Time& time)
{
  ROS_DEBUG_STREAM(
    "left_motor_velocity: " << left_motor_velocity_ << " right_motor_velocity: " << right_motor_velocity_);

  const double left_tangential_velocity = left_motor_velocity_ * 0.5 * config_.wheel_diameter;
  const double right_tangential_velocity = right_motor_velocity_ * 0.5 * config_.wheel_diameter;

  if (config_.publish_motor_speed)
  {
    publishDoubleValue(left_motor_velocity_, left_motor_speed_pub_);
    publishDoubleValue(right_motor_velocity_, right_motor_speed_pub_);
    publishDoubleValue(left_tangential_velocity, left_velocity_pub_);
    publishDoubleValue(right_tangential_velocity, right_velocity_pub_);
  }

  ROS_DEBUG_STREAM("left_tangential_velocity: " << left_tangential_velocity << " right_tangential_velocity: "
                                                << right_tangential_velocity);

  if (!std::isfinite(left_tangential_velocity) || !std::isfinite(right_tangential_velocity))
  {
    ROS_ERROR_STREAM("velocity of motors is not valid (NAN or INF): "
                       << " left_tangential_velocity: " << left_tangential_velocity
                       << " right_tangential_velocity: " << right_tangential_velocity);
    return;
  }

  linear_velocity_odom_ = (left_tangential_velocity + right_tangential_velocity) / 2.;
  angular_velocity_odom_ = (right_tangential_velocity - left_tangential_velocity) / config_.track_width;

  if (!odom_update_time_.isZero())
  {
    const double time_difference = (time - odom_update_time_).toSec();

    if (time_difference < 0.)
    {
      ROS_WARN("time difference for odom update is negative, skipping update");
      return;
    }

    x_odom_ += linear_velocity_odom_ * std::cos(yaw_odom_) * time_difference;
    y_odom_ += linear_velocity_odom_ * std::sin(yaw_odom_) * time_difference;
    yaw_odom_ = angles::normalize_angle(yaw_odom_ + angular_velocity_odom_ * time_difference);
  }

  odom_update_time_ = time;

  publishOdom();
}

void VescDifferentialDrive::publishOdom()
{
  const tf::Pose odom_pose(tf::createQuaternionFromYaw(yaw_odom_), tf::Vector3(x_odom_, y_odom_, 0.));

  if (config_.publish_odom)
  {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = odom_update_time_;
    odom_msg.header.frame_id = config_.odom_frame;
    odom_msg.child_frame_id = config_.base_frame;

    tf::poseTFToMsg(odom_pose, odom_msg.pose.pose);

    odom_msg.twist.twist.linear.x = linear_velocity_odom_;
    odom_msg.twist.twist.angular.z = angular_velocity_odom_;

    odom_pub_.publish(odom_msg);
  }

  if (config_.publish_tf)
  {
    const tf::StampedTransform transform(odom_pose, odom_update_time_, config_.odom_frame, config_.base_frame);
    tf_broadcaster_.sendTransform(transform);
  }
}

double VescDifferentialDrive::ensureBounds(double value, double max)
{
  return ensureBounds(value, -max, max);
}

double VescDifferentialDrive::ensureBounds(double value, double min, double max)
{
  return std::min(std::max(value, min), max);
}

void VescDifferentialDrive::publishDoubleValue(const double& value, ros::Publisher& publisher)
{
  std_msgs::Float32 msg;
  msg.data = static_cast<std_msgs::Float32::_data_type>(value);
  publisher.publish(msg);
}

}
