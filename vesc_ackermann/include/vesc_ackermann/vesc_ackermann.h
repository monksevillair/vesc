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
#include <vesc_ackermann/AxleConfig.h>
#include <vesc_ackermann/types.h>
#include <vesc_motor/vesc_transport_factory.h>

namespace vesc_ackermann
{
class VescAckermann
{
public:
  explicit VescAckermann(const ros::NodeHandle& private_nh);

protected:
  void commandVelocityCB(const ackermann_msgs::AckermannDriveConstPtr& cmd_vel);
  void calcOdomSpeed(const ros::Time& time);
  double getSupplyVoltage();

  void reconfigure(AckermannConfig& config, uint32_t level);
  void reconfigureFrontAxle(AxleConfig& config, uint32_t level);
  void reconfigureRearAxle(AxleConfig& config, uint32_t level);
  void reinitialize();

  void odomTimerCB(const ros::TimerEvent& event);
  void updateOdometry(const ros::Time& time);
  void publishOdom();
  double ensureBounds(double value, double max);
  double ensureBounds(double value, double min, double max);
  void publishDoubleValue(const double& value, ros::Publisher& publisher);

  ros::NodeHandle private_nh_;
  ros::NodeHandle front_axle_private_nh_;
  ros::NodeHandle rear_axle_private_nh_;

  AckermannConfig config_;
  AxleConfig front_axle_config_;
  AxleConfig rear_axle_config_;

  dynamic_reconfigure::Server<AckermannConfig> reconfigure_server_;
  dynamic_reconfigure::Server<AxleConfig> front_axle_reconfigure_server_;
  dynamic_reconfigure::Server<AxleConfig> rear_axle_reconfigure_server_;

  std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory_;

  AxlePtr front_axle_;
  AxlePtr rear_axle_;

  bool initialized_ = false;

  ros::Time odom_update_time_;

  VehicleVelocity velocity_odom_;

  double x_odom_ = 0.0;
  double y_odom_ = 0.0;
  double yaw_odom_ = 0.0;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher battery_voltage_pub_;

  ros::Subscriber cmd_vel_sub_;

  ros::Timer odom_timer_;
};
}


#endif //VESC_ACKERMANN_VESC_ACKERMANN_H
