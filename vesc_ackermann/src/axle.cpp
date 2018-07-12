/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_ackermann/axle.h>
#include <vesc_ackermann/steering.h>
#include <vesc_ackermann/vehicle.h>

namespace vesc_ackermann
{
Axle::Axle(const ros::NodeHandle& nh, const AckermannConfig& common_config, const MotorFactoryPtr& motor_factory)
  : nh_(nh), motor_factory_(motor_factory), common_config_(common_config), axle_config_server_(nh),
    steering_(std::make_shared<IdealAckermannSteering>(0.0))
{
  axle_config_server_.setCallback(std::bind(&Axle::reconfigure, this, std::placeholders::_1, std::placeholders::_2));
}

void Axle::reconfigure(const AxleConfig& axle_config, uint32_t /*level*/)
{
  axle_config_ = axle_config;

  if (axle_config_.wheel_diameter == 0.0)
  {
    ROS_ERROR("Parameter wheel_diameter is not set");
  }

  left_wheel_.position_x_ = axle_config_.position_x;
  left_wheel_.position_y_ = axle_config_.position_y + 0.5 * axle_config_.track;
  left_wheel_.hinge_position_y_ = axle_config_.position_y + 0.5 * axle_config_.track - axle_config_.steering_hinge_offset;
  left_wheel_.radius_ = 0.5 * axle_config_.wheel_diameter;
  left_wheel_.steering_ = steering_;

  right_wheel_.position_x_ = axle_config_.position_x;
  right_wheel_.position_y_ = axle_config_.position_y - 0.5 * axle_config_.track;
  right_wheel_.hinge_position_y_ = axle_config_.position_y - 0.5 * axle_config_.track + axle_config_.steering_hinge_offset;
  right_wheel_.radius_ = 0.5 * axle_config_.wheel_diameter;
  right_wheel_.steering_ = steering_;

  if (axle_config_.is_steered && !steering_motor_)
  {
    ros::NodeHandle steering_motor_nh(nh_, "steering_motor");
    steering_motor_.emplace(motor_factory_, steering_motor_nh, common_config_.publish_motor_speed);
  }
  else if (!axle_config_.is_steered && steering_motor_)
  {
    steering_motor_.reset();
  }

  if (axle_config_.is_driven && !left_motor_)
  {
    ros::NodeHandle left_motor_nh(nh_, "left_motor");
    left_motor_.emplace(motor_factory_, left_motor_nh, common_config_.publish_motor_speed);
  }
  else if (!axle_config_.is_driven && left_motor_)
  {
    left_motor_.reset();
  }

  if (axle_config_.is_driven && !right_motor_)
  {
    ros::NodeHandle right_motor_nh(nh_, "right_motor");
    right_motor_.emplace(motor_factory_, right_motor_nh, common_config_.publish_motor_speed);
  }
  else if (!axle_config_.is_driven && right_motor_)
  {
    right_motor_.reset();
  }
}

const AxleConfig& Axle::getConfig() const
{
  return axle_config_;
}

void Axle::setCommonConfig(const AckermannConfig& common_config)
{
  common_config_ = common_config;
}

void Axle::setVelocity(const double linear_velocity, const double normalized_steering_angle, const ros::Time& time)
{
  const double tan_normalized_steering_angle = std::tan(normalized_steering_angle);
  const double axle_steering_angle = std::atan2(tan_normalized_steering_angle * axle_config_.position_x, 1.0);
  const double angular_velocity = tan_normalized_steering_angle * linear_velocity;

  double current_steering_angle = 0.0;
  double current_steering_velocity = 0.0;

  if (steering_motor_)
  {
    current_steering_angle = steering_motor_->getPosition(time);

    const double position_difference = axle_steering_angle - current_steering_angle;
    if (position_difference > common_config_.steering_angle_tolerance)
    {
      current_steering_velocity = common_config_.steering_angle_velocity;
    }
    else if (position_difference < -common_config_.steering_angle_tolerance)
    {
      current_steering_velocity = -common_config_.steering_angle_velocity;
    }

    steering_motor_->setPosition(axle_steering_angle);
  }

  if (left_motor_ && right_motor_)
  {
    const double left_velocity =
      left_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, current_steering_angle,
                                       current_steering_velocity);
    const double right_velocity =
      right_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, current_steering_angle,
                                        current_steering_velocity);

    if ((std::fabs(left_velocity) <= common_config_.brake_velocity)
      && (std::fabs(right_velocity) <= common_config_.brake_velocity)
      && (std::fabs(left_motor_->getVelocity(time)) <= common_config_.allowed_brake_velocity)
      && (std::fabs(right_motor_->getVelocity(time)) <= common_config_.allowed_brake_velocity))
    {
      ROS_DEBUG_STREAM("VescDifferentialDrive::processAckermannCommand::7");

      ROS_DEBUG_STREAM("brake due to left_velocity: " << left_velocity << ", right_velocity: " << right_velocity);
      left_motor_->brake(common_config_.brake_current);
      right_motor_->brake(common_config_.brake_current);
    }
    else
    {
      ROS_DEBUG_STREAM("VescDifferentialDrive::processAckermannCommand::8");

      left_motor_->setVelocity(left_velocity);
      right_motor_->setVelocity(right_velocity);
    }
  }
}

void Axle::getVelocityConstraints(const ros::Time& time, VehicleVelocityConstraints& constraints)
{
  double steering_angle = 0.0;
  double steering_velocity = 0.0;
  if (steering_motor_)
  {
    steering_angle = steering_motor_->getPosition(time);
    steering_velocity = steering_motor_->getVelocity(time);
  }

  boost::optional<double> left_velocity;
  if (left_motor_)
  {
    left_velocity = left_motor_->getVelocity(time);
  }

  boost::optional<double> right_velocity;
  if (right_motor_)
  {
    right_velocity = right_motor_->getVelocity(time);
  }

  left_wheel_.computeVehicleVelocityConstraints(left_velocity, steering_angle, steering_velocity, constraints);
  right_wheel_.computeVehicleVelocityConstraints(right_velocity, steering_angle, steering_velocity, constraints);
}

void Axle::getJointStates(const ros::Time& time, sensor_msgs::JointState& joint_states)
{
  // TODO
}

boost::optional<double> Axle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  if (left_motor_)
  {
    supply_voltage += left_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (right_motor_)
  {
    supply_voltage += right_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (steering_motor_)
  {
    supply_voltage += steering_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }
  return boost::none;
}
}
