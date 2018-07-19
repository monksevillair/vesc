/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_BASE_CONTROL_AXLE_H
#define ARTI_BASE_CONTROL_AXLE_H

#include <ackermann_msgs/AckermannDrive.h>
#include <arti_base_control/AxleConfig.h>
#include <arti_base_control/steering.h>
#include <arti_base_control/types.h>
#include <arti_base_control/VehicleConfig.h>
#include <arti_base_control/wheel.h>
#include <boost/optional.hpp>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

namespace arti_base_control
{
class Axle
{
public:
  Axle(const ros::NodeHandle& nh, const VehicleConfig& vehicle_config, const MotorFactoryPtr& motor_factory);

  const AxleConfig& getConfig() const;
  void setVehicleConfig(const VehicleConfig& vehicle_config);

  void setVelocity(double linear_velocity, double angular_velocity, double axle_steering_angle, const ros::Time& time);

  void getVelocityConstraints(const ros::Time& time, VehicleVelocityConstraints& constraints);

  void getJointStates(const ros::Time& time, sensor_msgs::JointState& joint_states);

  boost::optional<double> getSupplyVoltage();

protected:
  void reconfigure(AxleConfig& config);

  ros::NodeHandle nh_;
  MotorFactoryPtr motor_factory_;

  VehicleConfig vehicle_config_;
  boost::optional<AxleConfig> config_;
  dynamic_reconfigure::Server<AxleConfig> reconfigure_server_;

  std::shared_ptr<IdealAckermannSteering> steering_;
  Wheel left_wheel_;
  Wheel right_wheel_;

  SteeringMotorPtr steering_motor_;
  DriveMotorPtr left_motor_;
  DriveMotorPtr right_motor_;
};
}

#endif //ARTI_BASE_CONTROL_AXLE_H
