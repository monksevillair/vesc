/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_ackermann/vesc_ackermann.h>
#include <angles/angles.h>
#include <functional>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

namespace vesc_ackermann
{
VescAckermann::VescAckermann(ros::NodeHandle private_nh)
  : private_nh_(private_nh), reconfigure_server_(private_nh_),
    transport_factory_(new vesc_motor::VescTransportFactory(private_nh_))
{
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::4");

  cmd_vel_sub_ = private_nh_.subscribe("/cmd_vel", 1, &VescAckermann::commandVelocityCB, this);
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::5");

  battery_voltage_pub_ = private_nh_.advertise<std_msgs::Float32>("/battery_voltage", 1);
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::7");

  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::1");
  reconfigure_server_.setCallback(boost::bind(&VescAckermann::reconfigure, this, _1, _2));

  initialized_ = true;
}

void VescAckermann::reconfigure(AckermannConfig& config, uint32_t /*level*/)
{
  if (config.max_velocity_linear == 0.0)
  {
    ROS_ERROR("Parameter max_velocity_linear is not set");
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

  if (!front_axis_ && !rear_axis_)
  {
    ros::NodeHandle front_axis_private_nh(private_nh_, "front_axis");
    ros::NodeHandle rear_axis_private_nh(private_nh_, "rear_axis");

    const double execution_duration = 1.0 / (config_.odometry_rate * 2.1);

    front_axis_ = std::make_shared<Axle>(
      front_axis_private_nh, transport_factory_, execution_duration, config_.publish_motor_speed, true,
      config_.steering_tolerance, config_.steering_angle_velocity, config_.max_steering_angle, config_.wheelbase,
      config_.wheel_diameter, config_.allowed_brake_velocity, config_.brake_velocity, config_.brake_current);

    rear_axis_ = std::make_shared<Axle>(
      rear_axis_private_nh, transport_factory_, execution_duration, config_.publish_motor_speed, false,
      config_.steering_tolerance, config_.steering_angle_velocity, config_.max_steering_angle, config_.wheelbase,
      config_.wheel_diameter, config_.allowed_brake_velocity, config_.brake_velocity, config_.brake_current);

    if (front_axis_->isSteered() && rear_axis_->isSteered())
    {
      front_axis_->halfWheelbase();
      rear_axis_->halfWheelbase();
    }
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }

  if (!odom_timer_)
  {
    odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate),
                                          &VescAckermann::odomTimerCB, this);
  }
}

void VescAckermann::odomTimerCB(const ros::TimerEvent& event)
{
  ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::1");

  if (initialized_)
  {
    updateOdometry(event.current_real);
    ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::5");

    publishDoubleValue(getSupplyVoltage(), battery_voltage_pub_);
    ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::6");
  }
}

void VescAckermann::updateOdometry(const ros::Time& time)
{
  calcOdomSpeed(time);

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

void VescAckermann::publishOdom()
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

double VescAckermann::ensureBounds(double value, double max)
{
  return ensureBounds(value, -max, max);
}

double VescAckermann::ensureBounds(double value, double min, double max)
{
  return std::min(std::max(value, min), max);
}

void VescAckermann::publishDoubleValue(const double& value, ros::Publisher& publisher)
{
  std_msgs::Float32 msg;
  msg.data = static_cast<std_msgs::Float32::_data_type>(value);
  publisher.publish(msg);
}

void VescAckermann::commandVelocityCB(const ackermann_msgs::AckermannDriveConstPtr& cmd_vel)
{
  if (!initialized_ || !front_axis_ || !rear_axis_)
    return;

  front_axis_->setSteeringAngle(cmd_vel->steering_angle);
  rear_axis_->setSteeringAngle(cmd_vel->steering_angle);

  boost::optional<double> wheel_base;
  wheel_base = front_axis_->getWheelbase();
  if (!wheel_base)
    wheel_base = rear_axis_->getWheelbase();

  const double rotation_speed = (cmd_vel->speed * std::tan(cmd_vel->steering_angle)) / wheel_base.get();

  front_axis_->setSpeed(cmd_vel->speed, rotation_speed);
  rear_axis_->setSpeed(cmd_vel->speed, rotation_speed);
}

double VescAckermann::getSupplyVoltage()
{
  boost::optional<Axle::DriveMotorHelper>& front_drive_motors = front_axis_->getDriveMotors();
  if (front_drive_motors)
    return front_drive_motors->left_motor.getSupplyVoltage();

  boost::optional<Axle::DriveMotorHelper>& rear_drive_motors = rear_axis_->getDriveMotors();
  if (rear_drive_motors)
    return rear_drive_motors->left_motor.getSupplyVoltage();

  return 0.;
}

void VescAckermann::calcOdomSpeed(const ros::Time& time)
{
  boost::optional<double> front_steering = front_axis_->getSteeringAngle(time);
  boost::optional<Axle::DriveMotorHelper>& front_drive_motors = front_axis_->getDriveMotors();

  double front_radius = calculateRadius(front_steering, front_drive_motors);

  double steering_front_left;
  double steering_front_right;
  calculateSteering(front_steering, front_drive_motors, front_radius, steering_front_left, steering_front_right);

  double front_left_velocity_correction = 0.;
  double front_right_velocity_correction = 0.;
  if (front_steering && old_front_steering_ && (time >= old_front_steering_time_))
  {
    double old_front_radius = calculateRadius(old_front_steering_, front_drive_motors);

    double old_steering_front_left;
    double old_steering_front_right;
    calculateSteering(old_front_steering_, front_drive_motors, old_front_radius, old_steering_front_left,
                      old_steering_front_right);

    double time_difference = (time - old_front_steering_time_).toSec();

    double delta_front_left = (steering_front_left - old_steering_front_left) / time_difference;
    double delta_front_right = (steering_front_right - old_steering_front_right) / time_difference;

    if (front_drive_motors)
    {
      front_left_velocity_correction = delta_front_left * front_drive_motors->wheel_rotation_offset;
      front_right_velocity_correction = -(delta_front_right * front_drive_motors->wheel_rotation_offset);
    }
  }
  old_front_steering_ = front_steering;
  old_front_steering_time_ = time;

  double front_left_x_wheel_offset;
  double front_left_y_wheel_offset;
  double front_right_x_wheel_offset;
  double front_right_y_wheel_offset;
  calculateOffset(steering_front_left, steering_front_right, front_drive_motors, front_left_x_wheel_offset,
                  front_left_y_wheel_offset, front_right_x_wheel_offset, front_right_y_wheel_offset);

  boost::optional<double> rear_steering = rear_axis_->getSteeringAngle(time);
  boost::optional<Axle::DriveMotorHelper>& rear_drive_motors = rear_axis_->getDriveMotors();

  double rear_radius = calculateRadius(rear_steering, rear_drive_motors);

  double steering_rear_left;
  double steering_rear_right;
  calculateSteering(rear_steering, rear_drive_motors, rear_radius, steering_rear_left, steering_rear_right);

  double rear_left_velocity_correction = 0.;
  double rear_right_velocity_correction = 0.;
  if (rear_steering && old_rear_steering_ && (time >= old_rear_steering_time_))
  {
    double old_rear_radius = calculateRadius(old_rear_steering_, rear_drive_motors);

    double old_steering_rear_left;
    double old_steering_rear_right;
    calculateSteering(old_rear_steering_, rear_drive_motors, old_rear_radius, old_steering_rear_left,
                      old_steering_rear_right);

    double time_difference = (time - old_rear_steering_time_).toSec();

    double delta_rear_left = (steering_rear_left - old_steering_rear_left) / time_difference;
    double delta_rear_right = (steering_rear_right - old_steering_rear_right) / time_difference;

    if (rear_drive_motors)
    {
      rear_left_velocity_correction = delta_rear_left * rear_drive_motors->wheel_rotation_offset;
      rear_right_velocity_correction = -(delta_rear_right * rear_drive_motors->wheel_rotation_offset);
    }
  }
  old_front_steering_ = front_steering;
  old_front_steering_time_ = time;

  double rear_left_x_wheel_offset;
  double rear_left_y_wheel_offset;
  double rear_right_x_wheel_offset;
  double rear_right_y_wheel_offset;
  calculateOffset(steering_rear_left, steering_rear_right, rear_drive_motors, rear_left_x_wheel_offset,
                  rear_left_y_wheel_offset, rear_right_x_wheel_offset, rear_right_y_wheel_offset);

  std::vector<double> rotation_velocities;
  std::vector<double> translation_velocities;

  boost::optional<double> front_left_velocity;
  boost::optional<double> front_right_velocity;
  if (front_drive_motors)
  {
    front_left_velocity = front_drive_motors->left_motor.getVelocity(time) + front_left_velocity_correction;
    front_right_velocity = front_drive_motors->right_motor.getVelocity(time) + front_right_velocity_correction;
  }

  boost::optional<double> rear_left_velocity;
  boost::optional<double> rear_right_velocity;
  if (rear_drive_motors)
  {
    rear_left_velocity = rear_drive_motors->left_motor.getVelocity(time) + rear_left_velocity_correction;
    rear_right_velocity = rear_drive_motors->right_motor.getVelocity(time) + rear_right_velocity_correction;
  }

  if (front_steering)
  {
    if (front_left_velocity)
    {
      double new_rotation_velocity = calculateLeftRotationVelocity(front_radius, front_drive_motors,
                                                                   front_left_y_wheel_offset,
                                                                   front_left_x_wheel_offset,
                                                                   front_left_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * front_radius);
    }

    if (front_right_velocity)
    {
      double new_rotation_velocity = calculateRightRotationVelocity(front_radius, front_drive_motors,
                                                                    front_right_y_wheel_offset,
                                                                    front_right_x_wheel_offset,
                                                                    front_right_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * front_radius);
    }

    if (rear_left_velocity)
    {
      double new_rotation_velocity = calculateLeftRotationVelocity(front_radius, rear_drive_motors,
                                                                   rear_left_y_wheel_offset,
                                                                   rear_left_x_wheel_offset,
                                                                   rear_left_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * front_radius);
    }

    if (rear_right_velocity)
    {
      double new_rotation_velocity = calculateRightRotationVelocity(front_radius, rear_drive_motors,
                                                                    rear_right_y_wheel_offset,
                                                                    rear_right_x_wheel_offset,
                                                                    rear_right_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * front_radius);
    }
  }

  if (rear_steering)
  {
    if (front_left_velocity)
    {
      double new_rotation_velocity = calculateLeftRotationVelocity(rear_radius, front_drive_motors,
                                                                   front_left_y_wheel_offset,
                                                                   front_left_x_wheel_offset,
                                                                   front_left_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * rear_radius);
    }

    if (front_right_velocity)
    {
      double new_rotation_velocity = calculateRightRotationVelocity(rear_radius, front_drive_motors,
                                                                    front_right_y_wheel_offset,
                                                                    front_right_x_wheel_offset,
                                                                    front_right_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * rear_radius);
    }

    if (rear_left_velocity)
    {
      double new_rotation_velocity = calculateLeftRotationVelocity(rear_radius, rear_drive_motors,
                                                                   rear_left_y_wheel_offset,
                                                                   rear_left_x_wheel_offset,
                                                                   rear_left_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * rear_radius);
    }

    if (rear_right_velocity)
    {
      double new_rotation_velocity = calculateRightRotationVelocity(rear_radius, rear_drive_motors,
                                                                    rear_right_y_wheel_offset,
                                                                    rear_right_x_wheel_offset,
                                                                    rear_right_velocity.get());
      rotation_velocities.push_back(new_rotation_velocity);
      translation_velocities.push_back(new_rotation_velocity * rear_radius);
    }
  }

  angular_velocity_odom_ = 0.;
  for (double new_rotation_velocity : rotation_velocities)
    angular_velocity_odom_ += new_rotation_velocity;
  angular_velocity_odom_ /= static_cast<double>(rotation_velocities.size());

  linear_velocity_odom_ = 0.;
  for (double new_translation_velocity : translation_velocities)
    linear_velocity_odom_ += new_translation_velocity;
  linear_velocity_odom_ /= static_cast<double>(translation_velocities.size());
}

void VescAckermann::calculateSteering(boost::optional<double> steering_center,
                                      const boost::optional<Axle::DriveMotorHelper>& drive_motors, double radius,
                                      double& steering_left, double& steering_right)
{
  if (!steering_center || !drive_motors)
  {
    steering_left = 0.;
    steering_right = 0.;
    return;
  }

  steering_left = std::atan2(radius - drive_motors->track_width / 2., drive_motors->distance_to_wheel_base);
  steering_right = std::atan2(radius + drive_motors->track_width / 2., drive_motors->distance_to_wheel_base);
}

double VescAckermann::calculateRadius(boost::optional<double> steering,
                                      const boost::optional<Axle::DriveMotorHelper>& drive_motors)
{
  if (!steering || !drive_motors)
    return 0;

  return drive_motors->distance_to_wheel_base / std::tan(steering.get());
}

void VescAckermann::calculateOffset(double steering_left, double steering_right,
                                    const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                    double& left_x_wheel_offset, double& left_y_wheel_offset,
                                    double& right_x_wheel_offset, double& right_y_wheel_offset)
{
  calculateOffset(steering_left, drive_motors, left_x_wheel_offset, left_y_wheel_offset);
  calculateOffset(steering_right, drive_motors, right_x_wheel_offset, right_y_wheel_offset);
}

void VescAckermann::calculateOffset(double steering, const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                    double& x_wheel_offset, double& y_wheel_offset)
{
  if (!drive_motors)
  {
    x_wheel_offset = 0.;
    y_wheel_offset = 0.;
    return;
  }

  x_wheel_offset = std::sin(steering) * drive_motors->wheel_rotation_offset;
  y_wheel_offset = std::sin(steering) * drive_motors->wheel_rotation_offset;
}

double VescAckermann::calculateLeftRotationVelocity(double radius,
                                                    const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                                    double left_y_wheel_offset, double left_x_wheel_offset,
                                                    double velocity)
{
  if (!drive_motors)
    return 0;

  const double y_offset = radius - drive_motors->track_width / 2. + left_y_wheel_offset;

  return calculateRotationVelocity(y_offset, drive_motors, left_x_wheel_offset, velocity);
}

double VescAckermann::calculateRightRotationVelocity(double radius,
                                                     const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                                     double left_y_wheel_offset, double left_x_wheel_offset,
                                                     double velocity)
{
  if (!drive_motors)
    return 0;

  const double y_offset = radius + drive_motors->track_width / 2. - left_y_wheel_offset;

  return calculateRotationVelocity(y_offset, drive_motors, left_x_wheel_offset, velocity);
}

double VescAckermann::calculateRotationVelocity(double y_offset,
                                                const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                                double left_x_wheel_offset, double velocity)
{
  if (!drive_motors)
    return 0;

  const double x_offset = drive_motors->distance_to_wheel_base + left_x_wheel_offset;
  const double distance = std::hypot(y_offset, x_offset);

  return velocity / distance;
}

}

