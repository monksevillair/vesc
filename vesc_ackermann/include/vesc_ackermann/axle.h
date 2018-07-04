/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_AXLE_H
#define VESC_ACKERMANN_AXLE_H

#include <boost/optional.hpp>
#include <memory>
#include <ros/node_handle.h>
#include <vesc_ackermann/vesc_drive_motor_decorator.h>
#include <vesc_ackermann/vesc_steering_motor_decorator.h>

namespace vesc_ackermann
{
class Axle
{
public:
  struct SteeringMotorHelper
  {
    SteeringMotorHelper(const ros::NodeHandle& private_nh,
                        std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                        double execution_duration, bool publish_motor_position, double _steering_velocity,
                        double _steering_tolerance, double _max_steering_angle, bool _is_fixed);

    double getSteeringAngle(const ros::Time& time);

    void setSteeringAngle(double steering_angle);

    double last_steering_angle = 0.0;
    double current_velocity = 0.0;
    double steering_velocity;
    double steering_tolerance;
    double max_steering_angle;
    VescSteeringMotorDecorator motor;
    bool is_fixed;
  };

  struct DriveMotorHelper
  {
    DriveMotorHelper(const ros::NodeHandle& left_motor_private_nh, const ros::NodeHandle& right_motor_private_nh,
                     std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                     double execution_duration, bool publish_motor_speed, double _distance_to_wheel_base,
                     double _track_width, double _wheel_rotation_offset);

    double distance_to_wheel_base;
    double track_width;
    double wheel_rotation_offset;

    VescDriveMotorDecorator left_motor;
    VescDriveMotorDecorator right_motor;
  };

  Axle(ros::NodeHandle nh, std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
       double execution_duration, bool publish_motor_state, bool front_axle, double steering_tolerance,
       double steering_angle_velocity, double max_steering_angle, double wheelbase, double wheel_diameter,
       double allowed_brake_velocity, double brake_velocity, double brake_current);

  boost::optional<double> getSteeringAngle(const ros::Time& time);
  boost::optional<DriveMotorHelper>& getDriveMotors();

  void setSteeringAngle(double steering_angle);
  void setSpeed(double translation_speed, double rotation_speed);

  bool isSteered();
  void halfWheelbase();

  boost::optional<double> getWheelbase();

private:
  bool front_axle_;

  boost::optional<SteeringMotorHelper> steering_motor_;

  boost::optional<DriveMotorHelper> drive_motor_;

  double wheel_diameter_;
  double allowed_brake_velocity_;
  double brake_velocity_;
  double brake_current_;
};
}

#endif //VESC_ACKERMANN_AXLE_H
