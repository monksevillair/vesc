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
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <vesc_ackermann/motor_factory.h>

namespace vesc_ackermann
{
VescAckermann::VescAckermann(const ros::NodeHandle& private_nh)
  : private_nh_(private_nh), reconfigure_server_(private_nh_)
{
  ROS_DEBUG_STREAM("VescAckermann::VescAckermann::4");

  cmd_vel_twist_sub_ = private_nh_.subscribe("cmd_vel", 1, &VescAckermann::processVelocityCommand, this);
  cmd_vel_ackermann_sub_ = private_nh_.subscribe("cmd_vel_ackermann", 1, &VescAckermann::processAckermannCommand, this);

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

void VescAckermann::reinitialize()
{
  if (!vehicle_)
  {
    vehicle_.emplace(private_nh_, std::make_shared<MotorFactory>(private_nh_, 1.0 / (config_.odometry_rate * 2.1)));
  }

  if (!odom_pub_ && config_.publish_odom)
  {
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  }
  else if (odom_pub_ && !config_.publish_odom)
  {
    odom_pub_.shutdown();
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
    odom_timer_ = private_nh_.createTimer(ros::Duration(1.0 / config_.odometry_rate), &VescAckermann::odomTimerCB,
                                          this);
  }
}

void VescAckermann::processVelocityCommand(const geometry_msgs::TwistConstPtr& cmd_vel)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_vel);
  }
}

void VescAckermann::processAckermannCommand(const ackermann_msgs::AckermannDriveConstPtr& cmd_vel)
{
  if (vehicle_)
  {
    vehicle_->setVelocity(*cmd_vel);
  }
}

void VescAckermann::odomTimerCB(const ros::TimerEvent& event)
{
  ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::1");

  if (initialized_)
  {
    updateOdometry(event.current_real);
    ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::5");
    publishOdom();

    publishSupplyVoltage();
    ROS_DEBUG_STREAM("VescAckermann::odomTimerCB::6");
  }
}

void VescAckermann::updateOdometry(const ros::Time& time)
{
  calcOdomSpeed(time);

  if (!odom_update_time_.isZero())
  {
    if (time >= odom_update_time_)
    {
      const double time_difference = (time - odom_update_time_).toSec();

      const double sin_yaw = std::sin(odom_pose_.theta);
      const double cos_yaw = std::cos(odom_pose_.theta);

      odom_pose_.x += (odom_velocity_.linear.x * cos_yaw - odom_velocity_.linear.y * sin_yaw) * time_difference;
      odom_pose_.y += (odom_velocity_.linear.x * sin_yaw + odom_velocity_.linear.y * cos_yaw) * time_difference;
      odom_pose_.theta = angles::normalize_angle(odom_pose_.theta + odom_velocity_.angular.z * time_difference);

      odom_update_time_ = time;
    }
    else
    {
      ROS_WARN("time difference for odom update is negative, skipping update");
    }
  }
  else
  {
    // Initialize time at first call:
    odom_update_time_ = time;
  }
}

void VescAckermann::calcOdomSpeed(const ros::Time& time)
{
  // Compute vehicle velocity using pseudo inverse of velocity constraints:
  VehicleVelocityConstraints constraints;
  front_axle_->getVelocityConstraints(time, constraints);
  rear_axle_->getVelocityConstraints(time, constraints);

  Eigen::MatrixXd a(Eigen::MatrixXd::Zero(constraints.size(), 3));
  Eigen::VectorXd b(Eigen::VectorXd::Zero(constraints.size()));
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    a(i, 0) = constraints[i].a_v_x;
    a(i, 1) = constraints[i].a_v_y;
    a(i, 2) = constraints[i].a_v_theta;
    b(i) = constraints[i].b;
  }

  Eigen::Vector3d x = a.fullPivHouseholderQr().solve(b);
  odom_velocity_.linear.x = x(0);
  odom_velocity_.linear.y = x(1);
  odom_velocity_.angular.z = x(2);
}

void VescAckermann::publishOdom()
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

void VescAckermann::publishSupplyVoltage()
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
    std_msgs::Float32 msg;
    msg.data = static_cast<std_msgs::Float32::_data_type>(supply_voltage / num_measurements);
    supply_voltage_pub_.publish(msg);
  }
}
}
