/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <arti_base_control/axle.h>
#include <arti_base_control/drive_motor.h>
#include <arti_base_control/motor_factory.h>
#include <arti_base_control/steering_motor.h>
#include <arti_base_control/utils.h>
#include <arti_base_control/vehicle.h>

namespace arti_base_control
{
Axle::Axle(const ros::NodeHandle& nh, const VehicleConfig& vehicle_config, const MotorFactoryPtr& motor_factory)
  : nh_(nh), motor_factory_(motor_factory), vehicle_config_(vehicle_config), reconfigure_server_(nh),
    steering_(std::make_shared<IdealAckermannSteering>(0.0))
{
  left_wheel_.steering_ = steering_;
  right_wheel_.steering_ = steering_;
  reconfigure_server_.setCallback(std::bind(&Axle::reconfigure, this, std::placeholders::_1));
}

void Axle::reconfigure(AxleConfig& config)
{
  if (config.wheel_diameter == 0.0)
  {
    ROS_ERROR("Parameter wheel_diameter is zero");
  }

  if (config_)
  {
    if (config.is_steered != config_->is_steered)
    {
      ROS_ERROR("Parameter is_steered cannot be changed dynamically");
      config.is_steered = config_->is_steered;
    }

    if (config.is_driven != config_->is_driven)
    {
      ROS_ERROR("Parameter is_driven cannot be changed dynamically");
      config.is_driven = config_->is_driven;
    }
  }
  else
  {
    // Initialize motors when callback is called for the first time (which happens when we call setCallback):
    if (config.is_steered)
    {
      ros::NodeHandle steering_motor_nh(nh_, "steering_motor");
      steering_motor_ = motor_factory_->createSteeringMotor(steering_motor_nh);
    }

    if (config.is_driven)
    {
      ros::NodeHandle left_motor_nh(nh_, "left_motor");
      left_motor_ = motor_factory_->createDriveMotor(left_motor_nh);

      ros::NodeHandle right_motor_nh(nh_, "right_motor");
      right_motor_ = motor_factory_->createDriveMotor(right_motor_nh);
    }
  }

  config_ = config;

  left_wheel_.position_x_ = config_->position_x;
  left_wheel_.position_y_ = config_->position_y + 0.5 * config_->track;
  left_wheel_.hinge_position_y_ = left_wheel_.position_y_ - config_->steering_hinge_offset;
  left_wheel_.radius_ = 0.5 * config_->wheel_diameter;

  right_wheel_.position_x_ = config_->position_x;
  right_wheel_.position_y_ = config_->position_y - 0.5 * config_->track;
  right_wheel_.hinge_position_y_ = right_wheel_.position_y_ + config_->steering_hinge_offset;
  right_wheel_.radius_ = 0.5 * config_->wheel_diameter;

  steering_->icr_x_ = config_->steering_icr_x;
}

const AxleConfig& Axle::getConfig() const
{
  if (config_)
  {
    return *config_;
  }
  throw std::logic_error("tried to get axle config before setting it");
}

void Axle::setVehicleConfig(const VehicleConfig& vehicle_config)
{
  vehicle_config_ = vehicle_config;
}

void Axle::setVelocity(const double linear_velocity, const double angular_velocity, const double axle_steering_angle,
                       const ros::Time& time)
{
  double current_steering_angle = 0.0;
  double current_steering_velocity = 0.0;

  if (steering_motor_)
  {
    current_steering_angle = steering_motor_->getPosition(time);

    const double position_difference = axle_steering_angle - current_steering_angle;
    if (position_difference > vehicle_config_.steering_angle_tolerance)
    {
      current_steering_velocity = vehicle_config_.steering_angle_velocity;
    }
    else if (position_difference < -vehicle_config_.steering_angle_tolerance)
    {
      current_steering_velocity = -vehicle_config_.steering_angle_velocity;
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

    if ((std::fabs(left_velocity) <= vehicle_config_.brake_velocity)
      && (std::fabs(right_velocity) <= vehicle_config_.brake_velocity)
      && (std::fabs(left_motor_->getVelocity(time)) <= vehicle_config_.allowed_brake_velocity)
      && (std::fabs(right_motor_->getVelocity(time)) <= vehicle_config_.allowed_brake_velocity))
    {
      left_motor_->brake(vehicle_config_.brake_current);
      right_motor_->brake(vehicle_config_.brake_current);
    }
    else
    {
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
  if (config_)
  {
    if (left_motor_ && !config_->left_wheel_joint.empty())
    {
      joint_states.name.push_back(config_->left_wheel_joint);
      joint_states.position.push_back(left_motor_->getPosition(time));
      joint_states.velocity.push_back(left_motor_->getVelocity(time));
    }

    if (right_motor_ && !config_->right_wheel_joint.empty())
    {
      joint_states.name.push_back(config_->right_wheel_joint);
      joint_states.position.push_back(right_motor_->getPosition(time));
      joint_states.velocity.push_back(right_motor_->getVelocity(time));
    }

    if (steering_motor_ && (!config_->left_hinge_joint.empty() || !config_->right_hinge_joint.empty()))
    {
      const double position = steering_motor_->getPosition(time);
      const double velocity = steering_motor_->getVelocity(time);

      if (!config_->left_hinge_joint.empty())
      {
        joint_states.name.push_back(config_->left_hinge_joint);
        joint_states.position.push_back(steering_->computeWheelSteeringAngle(left_wheel_, position));
        joint_states.velocity.push_back(steering_->computeWheelSteeringVelocity(left_wheel_, position, velocity));
      }

      if (!config_->right_hinge_joint.empty())
      {
        joint_states.name.push_back(config_->right_hinge_joint);
        joint_states.position.push_back(steering_->computeWheelSteeringAngle(right_wheel_, position));
        joint_states.velocity.push_back(steering_->computeWheelSteeringVelocity(right_wheel_, position, velocity));
      }
    }
  }
}

boost::optional<double> Axle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  if (left_motor_)
  {
    const boost::optional<double> motor_supply_voltage = left_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (right_motor_)
  {
    const boost::optional<double> motor_supply_voltage = right_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (steering_motor_)
  {
    const boost::optional<double> motor_supply_voltage =steering_motor_->getSupplyVoltage();
    if (motor_supply_voltage)
    {
      supply_voltage += *motor_supply_voltage;
      ++num_measurements;
    }
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }
  return boost::none;
}
}
