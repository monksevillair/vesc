/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_ackermann/axle.h>

namespace vesc_ackermann
{
Axle::SteeringMotorHelper::SteeringMotorHelper(
  const ros::NodeHandle& private_nh, std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
  double execution_duration, bool publish_motor_position, double _steering_velocity, double _steering_tolerance,
  double _max_steering_angle)
  : steering_velocity(_steering_velocity), steering_tolerance(_steering_tolerance),
    max_steering_angle(_max_steering_angle),
    motor(private_nh, transport_factory, execution_duration, publish_motor_position)
{
}

Axle::DriveMotorHelper::DriveMotorHelper(
  const ros::NodeHandle& left_motor_private_nh, const ros::NodeHandle& right_motor_private_nh,
  std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory, double execution_duration,
  bool publish_motor_speed, double _distance_to_wheel_base, double _track_width, double _wheel_rotation_offset)
  : distance_to_wheel_base(_distance_to_wheel_base), track_width(_track_width),
    wheel_rotation_offset(_wheel_rotation_offset),
    left_motor(left_motor_private_nh, transport_factory, execution_duration, publish_motor_speed),
    right_motor(right_motor_private_nh, transport_factory, execution_duration, publish_motor_speed)
{
}

Axle::Axle(ros::NodeHandle nh, const AxleConfig& config,
           std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
           double execution_duration, bool publish_motor_state, double steering_tolerance,
           double steering_angle_velocity, double max_steering_angle, double wheelbase,
           double allowed_brake_velocity, double brake_velocity, double brake_current)
  : config_(config), wheelbase_(wheelbase), allowed_brake_velocity_(allowed_brake_velocity),
    brake_velocity_(brake_velocity), brake_current_(brake_current)
{
  if (config_.is_steered)
  {
    const ros::NodeHandle steering_motor_private_nh(nh, "steering_motor");
    steering_motor_.emplace(steering_motor_private_nh, transport_factory, execution_duration, publish_motor_state,
                            steering_angle_velocity, steering_tolerance, max_steering_angle);
  }

  if (config_.is_driven)
  {
    const ros::NodeHandle left_motor_private_nh(nh, "left");
    const ros::NodeHandle right_motor_private_nh(nh, "right");

    drive_motor_.emplace(left_motor_private_nh, right_motor_private_nh, transport_factory, execution_duration,
                         publish_motor_state, wheelbase, track_width, wheel_rotation_offset);
  }
}

double Axle::getSteeringAngle(const ros::Time& time)
{
  if (steering_motor_)
  {
    return steering_motor_->getSteeringAngle(time);
  }
  return 0.0;
}

void Axle::setSteeringAngle(double steering_angle)
{
  if (steering_motor_)
  {
    steering_motor_->setSteeringAngle(steering_angle);
  }
}

boost::optional<Axle::DriveMotorHelper>& Axle::getDriveMotors()
{
  return drive_motor_;
}

void Axle::setSpeed(double translation_speed, double rotation_speed)
{
  if (!drive_motor_)
  {
    return;
  }

  double velocity_left;
  double velocity_right;

  if (steering_motor_)
  {
    const double tan_steering = std::tan(steering_motor_->last_steering_angle);
    const double radius = drive_motor_->distance_to_wheel_base / tan_steering;

    const double tw_half = drive_motor_->track_width / 2.;

    const double steering_angle_left = std::atan2(radius - tw_half, drive_motor_->distance_to_wheel_base);
    const double steering_angle_right = std::atan2(radius + tw_half, drive_motor_->distance_to_wheel_base);

    const double x_offset_left = -std::sin(steering_angle_left) * drive_motor_->wheel_rotation_offset;
    const double y_offset_left = std::cos(steering_angle_left) * drive_motor_->wheel_rotation_offset;

    const double x_offset_right = std::sin(steering_angle_right) * drive_motor_->wheel_rotation_offset;
    const double y_offset_right = -std::cos(steering_angle_right) * drive_motor_->wheel_rotation_offset;

    velocity_left = std::hypot(translation_speed - (tw_half + y_offset_left) * rotation_speed,
                               (drive_motor_->distance_to_wheel_base + x_offset_left) * rotation_speed);

    if ((translation_speed - (tw_half + y_offset_left) * rotation_speed) < 0.)
    {
      velocity_left *= -1.;
    }

    velocity_right = std::hypot(translation_speed + (tw_half + y_offset_right) * rotation_speed,
                                (drive_motor_->distance_to_wheel_base + x_offset_right) * rotation_speed);

    if ((translation_speed + (tw_half + y_offset_right) * rotation_speed) < 0.)
    {
      velocity_right *= -1.;
    }

    const double dt = 0.01;

    const double wb_twice = drive_motor_->distance_to_wheel_base * 2.;
    const double tan_new_steering
      = std::tan(steering_motor_->last_steering_angle + steering_motor_->current_velocity * dt);

    const double delta_alpha_left
      = (std::atan2((wb_twice - drive_motor_->track_width * tan_steering), (wb_twice * tan_steering))
        - std::atan2((wb_twice - drive_motor_->track_width * tan_new_steering), (wb_twice * tan_new_steering))) / dt;

    const double delta_alpha_right
      = (std::atan2((wb_twice + drive_motor_->track_width * tan_steering), (wb_twice * tan_steering))
        - std::atan2((wb_twice + drive_motor_->track_width * tan_new_steering), (wb_twice * tan_new_steering))) / dt;

    velocity_left -= delta_alpha_left * drive_motor_->wheel_rotation_offset;
    velocity_right += delta_alpha_right * drive_motor_->wheel_rotation_offset;
  }
  else
  {
    const double total_track_width = drive_motor_->track_width + drive_motor_->wheel_rotation_offset;

    velocity_left = translation_speed - total_track_width / 2. * rotation_speed;
    velocity_right = translation_speed + total_track_width / 2. * rotation_speed;
  }

  velocity_left *= wheel_diameter_;
  velocity_right *= wheel_diameter_;

  const ros::Time now = ros::Time::now();

  if ((std::fabs(velocity_left) <= brake_velocity_)
    && (std::fabs(velocity_right) <= brake_velocity_)
    && (std::fabs(drive_motor_->left_motor.getVelocity(now)) <= allowed_brake_velocity_)
    && (std::fabs(drive_motor_->right_motor.getVelocity(now)) <= allowed_brake_velocity_))
  {
    ROS_DEBUG_STREAM("VescDifferentialDrive::commandVelocityCB::7");

    ROS_DEBUG_STREAM("brake due to"
                       << " left_rotational_velocity: " << velocity_left
                       << " right_rotational_velocity: " << velocity_right);
    drive_motor_->left_motor.brake(brake_current_);
    drive_motor_->right_motor.brake(brake_current_);
  }
  else
  {
    ROS_DEBUG_STREAM("VescDifferentialDrive::commandVelocityCB::8");

    drive_motor_->left_motor.setVelocity(velocity_left);
    drive_motor_->right_motor.setVelocity(velocity_right);
  }
}

double Axle::SteeringMotorHelper::getSteeringAngle(const ros::Time& time)
{
  last_steering_angle = motor.getPosition(time);
  return last_steering_angle;
}

void Axle::SteeringMotorHelper::setSteeringAngle(double steering_angle)
{
  steering_angle = std::max(std::min(steering_angle, max_steering_angle), -max_steering_angle);

  const double position_difference = steering_angle - last_steering_angle;
  if (position_difference > steering_tolerance)
  {
    current_velocity = steering_velocity;
  }
  else if (position_difference < -steering_tolerance)
  {
    current_velocity = -steering_velocity;
  }
  else
  {
    current_velocity = 0.0;
  }

  motor.setPosition(steering_angle);
}
}
