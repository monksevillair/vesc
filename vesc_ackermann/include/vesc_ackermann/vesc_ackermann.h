/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_VESC_ACKERMANN_H
#define VESC_ACKERMANN_VESC_ACKERMANN_H

#include <ackermann_msgs/AckermannDrive.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <vesc_ackermann/AckermannConfig.h>
#include <vesc_ackermann/axle.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_ackermann
{
class VescAckermann
{
public:
  explicit VescAckermann(ros::NodeHandle private_nh);

protected:
  void commandVelocityCB(const ackermann_msgs::AckermannDriveConstPtr& cmd_vel);
  void calcOdomSpeed(const ros::Time& time);
  double getSupplyVoltage();

  void reconfigure(AckermannConfig& config, uint32_t level);
  void odomTimerCB(const ros::TimerEvent& event);
  void updateOdometry(const ros::Time& time);
  void publishOdom();
  double ensureBounds(double value, double max);
  double ensureBounds(double value, double min, double max);
  void publishDoubleValue(const double& value, ros::Publisher& publisher);

  void calculateSteering(boost::optional<double> steering_center,
                         const boost::optional<Axle::DriveMotorHelper>& drive_motors, double radius,
                         double& steering_left, double& steering_right);

  double calculateRadius(boost::optional<double> steering,
                         const boost::optional<Axle::DriveMotorHelper>& drive_motors);

  void calculateOffset(double steering_left, double steering_right,
                       const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                       double& left_x_wheel_offset, double& left_y_wheel_offset, double& right_x_wheel_offset,
                       double& right_y_wheel_offset);

  void calculateOffset(double steering, const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                       double& x_wheel_offset, double& y_wheel_offset);

  double calculateLeftRotationVelocity(double radius, const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                       double left_y_wheel_offset, double left_x_wheel_offset, double velocity);

  double calculateRightRotationVelocity(double radius, const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                        double left_y_wheel_offset, double left_x_wheel_offset, double velocity);

  double calculateRotationVelocity(double y_offset, const boost::optional<Axle::DriveMotorHelper>& drive_motors,
                                   double left_x_wheel_offset, double velocity);

  AckermannConfig config_;
  std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory_;

  std::shared_ptr<Axle> front_axis_;
  std::shared_ptr<Axle> rear_axis_;

  bool initialized_ = false;

  ros::NodeHandle private_nh_;
  dynamic_reconfigure::Server<AckermannConfig> reconfigure_server_;

  ros::Time odom_update_time_;

  double linear_velocity_odom_ = 0.0;
  double angular_velocity_odom_ = 0.0;

  double x_odom_ = 0.0;
  double y_odom_ = 0.0;
  double yaw_odom_ = 0.0;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher battery_voltage_pub_;

  ros::Subscriber cmd_vel_sub_;

  ros::Timer odom_timer_;

  boost::optional<double> old_front_steering_;
  ros::Time old_front_steering_time_;

  boost::optional<double> old_rear_steering_;
  ros::Time old_rear_steering_time_;
};
}


#endif //VESC_ACKERMANN_VESC_ACKERMANN_H
