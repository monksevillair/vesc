/*
Created by clemens on 6/25/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_BASE_CONTROL_BASE_CONTROL_H
#define ARTI_BASE_CONTROL_BASE_CONTROL_H

#include <ackermann_msgs/AckermannDrive.h>
#include <arti_base_control/BaseControlConfig.h>
#include <arti_base_control/types.h>
#include <arti_base_control/vehicle.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

namespace arti_base_control
{
class BaseControl
{
public:
  explicit BaseControl(const ros::NodeHandle& private_nh);

protected:
  void reconfigure(BaseControlConfig& config);

  void processVelocityCommand(const geometry_msgs::TwistConstPtr& cmd_vel);
  void processAckermannCommand(const ackermann_msgs::AckermannDriveConstPtr& cmd_ackermann);

  void odomTimerCB(const ros::TimerEvent& event);
  void updateOdometry(const ros::Time& time);
  void publishOdometry();

  void publishSupplyVoltage();

  ros::NodeHandle private_nh_;

  BaseControlConfig config_;
  dynamic_reconfigure::Server<BaseControlConfig> reconfigure_server_;

  boost::optional<Vehicle> vehicle_;

  ros::Time odom_update_time_;

  geometry_msgs::Pose2D odom_pose_;
  geometry_msgs::Twist odom_velocity_;

  ros::Publisher odom_pub_;
  boost::optional<tf::TransformBroadcaster> tf_broadcaster_;
  ros::Publisher joint_states_pub_;
  ros::Publisher supply_voltage_pub_;

  ros::Subscriber cmd_vel_twist_sub_;
  ros::Subscriber cmd_ackermann_sub_;

  ros::Timer odom_timer_;
};
}


#endif //ARTI_BASE_CONTROL_BASE_CONTROL_H
