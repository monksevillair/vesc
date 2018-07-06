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
VescAckermann::VescAckermann(const ros::NodeHandle& private_nh)
  : private_nh_(private_nh), front_axle_private_nh_(private_nh_, "front_axle"),
    rear_axle_private_nh_(private_nh_, "rear_axle"), reconfigure_server_(private_nh_),
    front_axle_reconfigure_server_(front_axle_private_nh_), rear_axle_reconfigure_server_(rear_axle_private_nh_),
    transport_factory_(std::make_shared<vesc_motor::VescTransportFactory>(private_nh_)), velocity_odom_(0.0, 0.0)
{
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::4");

  cmd_vel_sub_ = private_nh_.subscribe("/cmd_vel", 1, &VescAckermann::commandVelocityCB, this);
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::5");

  battery_voltage_pub_ = private_nh_.advertise<std_msgs::Float32>("/battery_voltage", 1);
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::7");

  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::1");
  reconfigure_server_.setCallback(boost::bind(&VescAckermann::reconfigure, this, _1, _2));
  front_axle_reconfigure_server_.setCallback(boost::bind(&VescAckermann::reconfigureFrontAxle, this, _1, _2));
  rear_axle_reconfigure_server_.setCallback(boost::bind(&VescAckermann::reconfigureRearAxle, this, _1, _2));

  reinitialize();
  initialized_ = true;
}

void VescAckermann::reconfigure(AckermannConfig& config, uint32_t /*level*/)
{
  if (config.max_velocity_linear == 0.0)
  {
    ROS_ERROR("Parameter max_velocity_linear is not set");
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

  if (initialized_)
  {
    reinitialize();
  }
}

void VescAckermann::reconfigureFrontAxle(AxleConfig& config, uint32_t /*level*/)
{
  if (config.wheel_diameter == 0.0)
  {
    ROS_ERROR("Parameter wheel_diameter is not set");
  }

  front_axle_config_ = config;

  if (initialized_)
  {
    reinitialize();
  }
}

void VescAckermann::reconfigureRearAxle(AxleConfig& config, uint32_t /*level*/)
{
  if (config.wheel_diameter == 0.0)
  {
    ROS_ERROR("Parameter wheel_diameter is not set");
  }

  rear_axle_config_ = config;

  if (initialized_)
  {
    reinitialize();
  }
}

void VescAckermann::reinitialize()
{
  if (!front_axle_)
  {
    double position_x = 0.0;
    if (front_axle_config_.is_steered && rear_axle_config_.is_steered)
    {
      position_x = config_.wheelbase * (1.0 - config_.icr_line_relative_position);
    }
    else if (front_axle_config_.is_steered)
    {
      position_x = config_.wheelbase;
    }

    front_axle_ = std::make_shared<Axle>(front_axle_private_nh_, config_, front_axle_config_, transport_factory_,
                                         position_x);
  }

  if (!rear_axle_)
  {
    double position_x = 0.0;
    if (front_axle_config_.is_steered && rear_axle_config_.is_steered)
    {
      position_x = -config_.wheelbase * config_.icr_line_relative_position;
    }
    else if (rear_axle_config_.is_steered)
    {
      position_x = -config_.wheelbase;
    }

    rear_axle_ = std::make_shared<Axle>(rear_axle_private_nh_, config_, rear_axle_config_, transport_factory_,
                                        position_x);
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }
  else if (odom_pub_ && !config_.publish_odom)
  {
    odom_pub_.shutdown();
  }

  if (!odom_timer_)
  {
    odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate), &VescAckermann::odomTimerCB,
                                          this);
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

    if (time_difference < 0.0)
    {
      ROS_WARN("time difference for odom update is negative, skipping update");
      return;
    }

    x_odom_ += velocity_odom_.linear_velocity * std::cos(yaw_odom_) * time_difference;
    y_odom_ += velocity_odom_.linear_velocity * std::sin(yaw_odom_) * time_difference;
    yaw_odom_ = angles::normalize_angle(yaw_odom_ + velocity_odom_.angular_velocity * time_difference);
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

    odom_msg.twist.twist.linear.x = velocity_odom_.linear_velocity;
    odom_msg.twist.twist.angular.z = velocity_odom_.angular_velocity;

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
  if (initialized_)
  {
    // TODO: this assumes that the steering velocity in the message is related to the vehicle wheelbase. Is this correct?
    const double steering_angle = std::max(std::min<double>(cmd_vel->steering_angle, config_.max_steering_angle),
                                           -config_.max_steering_angle);
    const double normalized_steering_angle = std::atan2(std::tan(steering_angle), config_.wheelbase);

    const ros::Time now = ros::Time::now();
    front_axle_->setVelocity(cmd_vel->speed, normalized_steering_angle, now);
    rear_axle_->setVelocity(cmd_vel->speed, normalized_steering_angle, now);
  }
}

double VescAckermann::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  const boost::optional<double> front_supply_voltage = front_axle_->getSupplyVoltage();
  if (front_supply_voltage)
  {
    supply_voltage += *front_supply_voltage;
    ++num_measurements;
  }

  const boost::optional<double> rear_supply_voltage = rear_axle_->getSupplyVoltage();
  if (rear_supply_voltage)
  {
    supply_voltage += *rear_supply_voltage;
    ++num_measurements;
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void VescAckermann::calcOdomSpeed(const ros::Time& time)
{
  std::vector<VehicleVelocity> velocities;

  if (front_axle_config_.is_driven)
  {
    const VehicleVelocity front_axle_velocity = *front_axle_->getVelocity(time);

    if (front_axle_config_.is_steered)
    {
      velocities.emplace_back(front_axle_velocity);
    }
    else if (rear_axle_config_.is_steered)
    {
      const double tan_rear_steering_angle = std::tan(rear_axle_->getSteeringAngle(time));
      velocities.emplace_back(front_axle_velocity.linear_velocity,
                              front_axle_velocity.linear_velocity * tan_rear_steering_angle / -config_.wheelbase);
    }
  }

  if (rear_axle_config_.is_driven)
  {
    const VehicleVelocity rear_axle_velocity = *rear_axle_->getVelocity(time);

    if (rear_axle_config_.is_steered)
    {
      velocities.emplace_back(rear_axle_velocity);
    }
    else if (front_axle_config_.is_steered)
    {
      const double tan_front_steering_angle = std::tan(front_axle_->getSteeringAngle(time));
      velocities.emplace_back(rear_axle_velocity.linear_velocity,
                              rear_axle_velocity.linear_velocity * tan_front_steering_angle / config_.wheelbase);
    }
  }

  VehicleVelocity total_velocity(0.0, 0.0);
  for (const VehicleVelocity& velocity : velocities)
  {
    total_velocity.linear_velocity += velocity.linear_velocity;
    total_velocity.angular_velocity += velocity.angular_velocity;
  }
  total_velocity.linear_velocity /= velocities.size();
  total_velocity.angular_velocity /= velocities.size();

  velocity_odom_ = total_velocity;
}
}
