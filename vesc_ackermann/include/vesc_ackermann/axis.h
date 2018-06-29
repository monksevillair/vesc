/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_ACKERMANN_AXIS_H
#define VESC_ACKERMANN_AXIS_H

#include <ros/ros.h>
#include <memory>
#include <vesc_ackermann/vesc_drive_motor_decorator.h>
#include <vesc_ackermann/vesc_steering_motor_decorator.h>
#include <boost/optional.hpp>

namespace vesc_ackermann
{
  class Axis
  {
  public:
    struct SteeringMotorHelper
    {
      double last_position;
      double current_velocity;
      double steering_velocity;
      double steering_tollerance;
      double max_steering_angular;
      VescSteeringMotorDecorator motor;
      bool is_fixed;

      SteeringMotorHelper(const ros::NodeHandle& private_nh,
                          std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                          double execution_duration, bool publish_motor_position, double _steering_velocity,
                          double _steering_tollerance, double _max_steering_angular, bool _is_fixed) :
          last_position(0.), current_velocity(0.), steering_velocity(_steering_velocity),
          steering_tollerance(_steering_tollerance), max_steering_angular(_max_steering_angular),
          motor(private_nh, transport_factory, execution_duration, publish_motor_position), is_fixed(_is_fixed)
      { }

      double getSteeringAngular(const ros::Time &time);

      void setSteeringAngular(double steering_angular);
    };

    struct DriveMotorHelper
    {
      double distance_to_wheel_base;
      double track_width;
      double wheel_rotation_offset;

      DriveMotorHelper(const ros::NodeHandle& left_motor_private_nh, const ros::NodeHandle& right_motor_private_nh,
                       std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                       double execution_duration, bool publish_motor_speed, double _distance_to_wheel_base,
                       double _track_width, double _wheel_rotation_offset) :
          distance_to_wheel_base(_distance_to_wheel_base), track_width(_track_width),
          wheel_rotation_offset(_wheel_rotation_offset),
          left_motor(left_motor_private_nh, transport_factory, execution_duration, publish_motor_speed),
          right_motor(right_motor_private_nh, transport_factory, execution_duration, publish_motor_speed)
      { }

      VescDriveMotorDecorator left_motor;
      VescDriveMotorDecorator right_motor;
    };

    Axis(ros::NodeHandle nh, std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
         double execution_duration, bool publish_motor_state, bool front_axis, double steering_tollerance,
         double steering_angular_velocity, double max_steering_angular, double wheel_base, double wheel_diameter,
         double allowed_brake_velocity, double brake_velocity, double brake_current);

    boost::optional<double> getSteeringAngular(const ros::Time &time);
    boost::optional<DriveMotorHelper>& getDriveMotors();

    void setSteeringAngular(double steering_angular);
    void setSpeed(double translation_speed, double rotation_speed);

    bool isSteered();
    void halfWheelBase();

    boost::optional<double> getWheelBase();

  private:
    bool front_axis_;

    boost::optional<SteeringMotorHelper> steering_motor_;

    boost::optional<DriveMotorHelper> drive_motor_;

    double wheel_diameter_;
    double allowed_brake_velocity_;
    double brake_velocity_;
    double brake_current_;

  };
}

#endif //VESC_ACKERMANN_AXIS_H
