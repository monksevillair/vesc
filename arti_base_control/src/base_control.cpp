/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_base_control/base_control.h>
#include <angles/angles.h>
#include <arti_base_control/motor_factory.h>
#include <functional>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

namespace arti_base_control
{
BaseControl::BaseControl(const ros::NodeHandle& private_nh)
  : private_nh_(private_nh), reconfigure_server_(private_nh_)
{
  cmd_vel_twist_sub_ = private_nh_.subscribe("cmd_vel", 1, &BaseControl::processVelocityCommand, this);
  cmd_ackermann_sub_ = private_nh_.subscribe("cmd_ackermann", 1, &BaseControl::processAckermannCommand, this);

  reconfigure_server_.setCallback(
    std::bind(&BaseControl::reconfigure, this, std::placeholders::_1));
}

void BaseControl::reconfigure(BaseControlConfig& config)
{
  config_ = config;

  if (!vehicle_)
  {
    vehicle_.emplace(
      ros::NodeHandle(private_nh_, "vehicle"),
      std::make_shared<MotorFactory>(private_nh_, 1.0 / (config_.odometry_rate * 2.1), config_.publish_motor_states,
                                     config_.use_mockup));
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }
  else if (odom_pub_ && !config_.publish_odom)
  {
    odom_pub_.shutdown();
  }

  if (!tf_broadcaster_ && config_.publish_tf)
  {
    tf_broadcaster_.emplace();
  }
  else if (tf_broadcaster_ && !config_.publish_tf)
  {
    tf_broadcaster_.reset();
  }

  if (!supply_voltage_pub_ && config_.publish_supply_voltage)
  {
    supply_voltage_pub_ = private_nh_.advertise<std_msgs::Float32>("supply_voltage", 1);
  }
  else if (supply_voltage_pub_ && !config_.publish_supply_voltage)
  {
    supply_voltage_pub_.shutdown();
  }

  if (!joint_states_pub_ && config_.publish_joint_states)
  {
    joint_states_pub_ = private_nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  }
  else if (joint_states_pub_ && !config_.publish_joint_states)
  {
    joint_states_pub_.shutdown();
  }

  if (!odom_timer_)
  {
    odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate), &BaseControl::odomTimerCB,
                                          this);
  }
}

void BaseControl::processVelocityCommand(const geometry_msgs::TwistConstPtr& cmd_vel)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_vel, ros::Time::now());
  }
}

void BaseControl::processAckermannCommand(const ackermann_msgs::AckermannDriveConstPtr& cmd_ackermann)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_ackermann, ros::Time::now());
  }
}

void BaseControl::odomTimerCB(const ros::TimerEvent& event)
{
  if (vehicle_)
  {
    updateOdometry(event.current_real);

    if (config_.publish_odom || config_.publish_tf)
    {
      publishOdometry();
    }

    if (config_.publish_supply_voltage)
    {
      publishSupplyVoltage();
    }

    if (config_.publish_joint_states)
    {
      joint_states_pub_.publish(vehicle_->getJointStates(event.current_real));
    }
  }
}

void BaseControl::updateOdometry(const ros::Time& time)
{
  if (time >= odom_update_time_)
  {
    odom_velocity_ = vehicle_->getVelocity(time);

    if (!odom_update_time_.isZero())
    {
      const double time_difference = (time - odom_update_time_).toSec();

      const double sin_yaw = std::sin(odom_pose_.theta);
      const double cos_yaw = std::cos(odom_pose_.theta);

      odom_pose_.x += (odom_velocity_.linear.x * cos_yaw - odom_velocity_.linear.y * sin_yaw) * time_difference;
      odom_pose_.y += (odom_velocity_.linear.x * sin_yaw + odom_velocity_.linear.y * cos_yaw) * time_difference;
      odom_pose_.theta = angles::normalize_angle(odom_pose_.theta + odom_velocity_.angular.z * time_difference);
    }

    odom_update_time_ = time;
  }
  else
  {
    ROS_WARN("time difference for odom update is negative, skipping update");
  }
}

void BaseControl::publishOdometry()
{
  if (!odom_update_time_.isZero())
  {
    const tf::Pose pose(tf::createQuaternionFromYaw(odom_pose_.theta), tf::Vector3(odom_pose_.x, odom_pose_.y, 0.0));

    if (config_.publish_odom)
    {
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = odom_update_time_;
      odom_msg.header.frame_id = config_.odom_frame;
      odom_msg.child_frame_id = config_.base_frame;

      tf::poseTFToMsg(pose, odom_msg.pose.pose);

      odom_msg.twist.twist = odom_velocity_;

      odom_pub_.publish(odom_msg);
    }

    if (config_.publish_tf)
    {
      const tf::StampedTransform transform(pose, odom_update_time_, config_.odom_frame, config_.base_frame);
      tf_broadcaster_->sendTransform(transform);
    }
  }
}

void BaseControl::publishSupplyVoltage()
{
  const boost::optional<double> supply_voltage = vehicle_->getSupplyVoltage();

  if (supply_voltage)
  {
    std_msgs::Float32 msg;
    msg.data = static_cast<std_msgs::Float32::_data_type>(*supply_voltage);
    supply_voltage_pub_.publish(msg);
  }
}
}
