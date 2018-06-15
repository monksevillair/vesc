/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DIFFERENTIAL_DRIVE_VESC_DIFFERENTIAL_DRIVE_H
#define VESC_DIFFERENTIAL_DRIVE_VESC_DIFFERENTIAL_DRIVE_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <vesc_differential_drive/DifferentialDriveConfig.h>
#include <vesc_differential_drive/vesc_motor.h>

namespace vesc_differential_drive
{
class VescDifferentialDrive
{
public:
  VescDifferentialDrive(ros::NodeHandle private_nh,
                        const ros::NodeHandle &left_motor_private_nh, const ros::NodeHandle &right_motor_private_nh);

private:
  void reconfigure(DifferentialDriveConfig& config, uint32_t level);
  void commandVelocityCB(const geometry_msgs::Twist &cmd_vel);
  void odomTimerCB(const ros::TimerEvent& event);
  void updateOdometry(const ros::Time &time);
  void publishOdom();
  double ensureBounds(double value, double max);
  double ensureBounds(double value, double min, double max);
  void publishDoubleValue(const double &value, ros::Publisher &publisher);

  bool initialized_;

  ros::NodeHandle private_nh_;
  dynamic_reconfigure::Server<DifferentialDriveConfig> reconfigure_server_;
  DifferentialDriveConfig config_;

  VescMotor left_motor_;
  double left_motor_velocity_;

  VescMotor right_motor_;
  double right_motor_velocity_;

  ros::Time odom_update_time_;

  double linear_velocity_odom_;
  double angular_velocity_odom_;

  double x_odom_;
  double y_odom_;
  double yaw_odom_;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher battery_voltage_pub_;

  ros::Publisher left_motor_speed_pub_;
  ros::Publisher right_motor_speed_pub_;
  ros::Publisher left_velocity_pub_;
  ros::Publisher right_velocity_pub_;

  ros::Publisher left_motor_speed_send_pub_;
  ros::Publisher right_motor_speed_send_pub_;
  ros::Publisher left_velocity_send_pub_;
  ros::Publisher right_velocity_send_pub_;

  ros::Subscriber cmd_vel_sub_;

  ros::Timer odom_timer_;
};
}

#endif // VESC_DIFFERENTIAL_DRIVE_VESC_DIFFERENTIAL_DRIVE_H
