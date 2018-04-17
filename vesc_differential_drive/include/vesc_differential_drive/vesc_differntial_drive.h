/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DIFFERENTIAL_DRIVE_VESCDIFFERNTIALDRIVE_H
#define VESC_DIFFERENTIAL_DRIVE_VESCDIFFERNTIALDRIVE_H

#include <vesc_differential_drive/vesc_motor.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace vesc_differntial_drive
{
class VescDifferntialDrive
{
public:
  VescDifferntialDrive(ros::NodeHandle nh, ros::NodeHandle private_nh,
                       const ros::NodeHandle &left_motor_private_nh, const ros::NodeHandle &right_motor_private_nh);

  void leftMotorSpeed(const double& speed, const ros::Time &time);

  void rightMotorSpeed(const double& speed, const ros::Time &time);

  void batteryVoltage(const double& voltage);

  void commandVelocityCB(const geometry_msgs::Twist &cmd_vel);

private:
  ros::NodeHandle nh_;

  VescMotor left_motor_;
  bool has_left_motor_speed_;
  double left_motor_speed_;

  VescMotor right_motor_;
  bool has_right_motor_speed_;
  double right_motor_speed_;

  double max_velocity_linear_;
  double max_velocity_angular_;
  double track_width_;
  double velocity_correction_left_;
  double velocity_correction_right_;
  double wheel_diameter_;

  double allowed_brake_rpms_;
  double brake_rpms_;
  double brake_current_;

  ros::Time odom_update_time_;

  double linear_velocity_odom_;
  double angular_velocity_odom_;

  double x_odom_;
  double y_odom_;
  double yaw_odom_;

  std::string odom_frame_;
  std::string base_frame_;

  bool publish_odom_;
  ros::Publisher odom_pub_;

  bool publish_tf_;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Publisher battery_voltage_pub_;

  ros::Subscriber cmd_vel_sub_;

  ros::Timer timer_;

  void timerCB(const ros::TimerEvent& event);

  void updateOdometry(const ros::Time time);

  void publishOdom();

  double ensurBounds(double value, double max);
  double ensurBounds(double value, double min, double max);
};
}

#endif //VESC_DIFFERENTIAL_DRIVE_VESCDIFFERNTIALDRIVE_H
