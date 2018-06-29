/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_ackermann/axis.h>

namespace vesc_ackermann
{

  Axis::Axis(ros::NodeHandle nh, std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
             double execution_duration, bool publish_motor_state, bool front_axis, double steering_tollerance,
             double steering_angular_velocity, double max_steering_angular, double wheel_base, double wheel_diameter,
             double allowed_brake_velocity, double brake_velocity, double brake_current) :
      front_axis_(front_axis), wheel_diameter_(wheel_diameter), allowed_brake_velocity_(allowed_brake_velocity),
      brake_velocity_(brake_velocity), brake_current_(brake_current)
  {
    bool is_steered = nh.param<bool>("is_steered", false);
    bool is_fixed = nh.param<bool>("is_fixed", false);
    if (is_steered || is_fixed)
    {
      ros::NodeHandle private_nh(nh.getNamespace() + "/steering");
      steering_motor_.emplace(private_nh, transport_factory, execution_duration, publish_motor_state,
                              steering_angular_velocity, steering_tollerance, max_steering_angular, is_fixed);
    }
    else
      wheel_base = 0.;

    bool is_driven = nh.param<bool>("is_driven", false);
    if (is_driven)
    {
      double track_width = nh.param<double>("track_width", 0.);
      double wheel_rotation_offset = nh.param<double>("wheel_rotation_offset", 0.);

      ros::NodeHandle left_motor_private_nh(nh.getNamespace() + "/left");
      ros::NodeHandle right_motor_private_nh(nh.getNamespace() + "/right");

      drive_motor_.emplace(left_motor_private_nh, right_motor_private_nh, transport_factory, execution_duration,
                           publish_motor_state, wheel_base, track_width, wheel_rotation_offset);
    }

  }

  boost::optional<double> Axis::getSteeringAngular(const ros::Time &time)
  {
    if (!steering_motor_ || steering_motor_->is_fixed)
      return boost::none;

    return steering_motor_->getSteeringAngular(time);
  }

  boost::optional<Axis::DriveMotorHelper>& Axis::getDriveMotors()
  {
    return drive_motor_;
  }

  void Axis::setSteeringAngular(double steering_angular)
  {
    if (steering_motor_)
    {
      if (steering_motor_->is_fixed)
        steering_motor_->setSteeringAngular(0.);
      else
        steering_motor_->setSteeringAngular(steering_angular * (front_axis_ ? 1. : -1.));
    }
  }

  void Axis::setSpeed(double translation_speed, double rotation_speed)
  {
    if (!drive_motor_)
      return;

    double velocity_left;
    double velocity_right;

    if (steering_motor_)
    {
      const double tan_steering = std::tan(steering_motor_->last_position);
      const double radius = drive_motor_->distance_to_wheel_base / tan_steering;

      const double tw_half = drive_motor_->track_width/2.;

      double steering_angular_left = std::atan2(radius - tw_half, drive_motor_->distance_to_wheel_base);
      double steering_angular_right = std::atan2(radius - tw_half, drive_motor_->distance_to_wheel_base);

      const double x_offset_left = -std::sin(steering_angular_left) * drive_motor_->wheel_rotation_offset;
      const double y_offset_left = std::cos(steering_angular_left) * drive_motor_->wheel_rotation_offset;

      const double x_offset_right = std::sin(steering_angular_right) * drive_motor_->wheel_rotation_offset;
      const double y_offset_right = -std::cos(steering_angular_right) * drive_motor_->wheel_rotation_offset;

      velocity_left = std::hypot(translation_speed - (tw_half + y_offset_left) * rotation_speed,
                                 (drive_motor_->distance_to_wheel_base + x_offset_left) * rotation_speed);
      if ((translation_speed - (tw_half + y_offset_left) * rotation_speed) < 0.)
        velocity_left *= -1.;

      velocity_right = std::hypot(translation_speed + (tw_half + y_offset_right) * rotation_speed,
                                  (drive_motor_->distance_to_wheel_base + x_offset_right) * rotation_speed);
      if ((translation_speed + (tw_half + y_offset_right) * rotation_speed) < 0.)
        velocity_right *= -1.;

      const double dt = 0.01;

      const double wb_twice = drive_motor_->distance_to_wheel_base * 2.;
      const double tan_new_steering = std::tan(steering_motor_->last_position + steering_motor_->current_velocity * dt);
      const double delta_alpha_left = (std::atan2((wb_twice - drive_motor_->track_width * tan_steering),
                                            (wb_twice * tan_steering)) -
                                std::atan2((wb_twice - drive_motor_->track_width * tan_new_steering),
                                           (wb_twice * tan_new_steering))) / dt;


      const double delta_alpha_right = (std::atan2((wb_twice + drive_motor_->track_width * tan_steering),
                                            (wb_twice * tan_steering)) -
                                 std::atan2((wb_twice + drive_motor_->track_width * tan_new_steering),
                                            (wb_twice * tan_new_steering))) / dt;

      velocity_left -= delta_alpha_left * drive_motor_->wheel_rotation_offset;
      velocity_right += delta_alpha_right * drive_motor_->wheel_rotation_offset;
    }
    else
    {
      const double total_track_width = drive_motor_->track_width + drive_motor_->wheel_rotation_offset;

      velocity_left = translation_speed - total_track_width/2. * rotation_speed;

      velocity_right = translation_speed + total_track_width/2. * rotation_speed;
    }

    velocity_left *= wheel_diameter_;
    velocity_right *= wheel_diameter_;

    ros::Time now = ros::Time::now();
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

  bool Axis::isSteered()
  {
    return static_cast<bool>(steering_motor_);
  }

  void Axis::halfWheelBase()
  {
    if (drive_motor_)
      drive_motor_->distance_to_wheel_base /= 2.;
  }

  boost::optional<double> Axis::getWheelBase()
  {
    if (drive_motor_)
      return drive_motor_->distance_to_wheel_base;

    return boost::none;
  }


  double Axis::SteeringMotorHelper::getSteeringAngular(const ros::Time &time)
  {
    last_position = motor.getPosition(time);
    return last_position;
  }

  void Axis::SteeringMotorHelper::setSteeringAngular(double steering_angular)
  {
    steering_angular = std::max(std::min(steering_angular, max_steering_angular), -max_steering_angular);

    double position_difference = steering_angular - last_position;
    if ((position_difference > 0.) && (position_difference > steering_tollerance))
      current_velocity = steering_velocity;
    else if ((position_difference < 0.) && (-position_difference > steering_tollerance))
      current_velocity = steering_velocity;
    else
      current_velocity = 0.;

    motor.setPosition(steering_angular);
  }
}