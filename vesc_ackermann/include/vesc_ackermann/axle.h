/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_AXLE_H
#define VESC_ACKERMANN_AXLE_H

#include <ackermann_msgs/AckermannDrive.h>
#include <boost/optional.hpp>
#include <memory>
#include <ros/node_handle.h>
#include <vesc_ackermann/AckermannConfig.h>
#include <vesc_ackermann/AxleConfig.h>
#include <vesc_ackermann/drive_motor.h>
#include <vesc_ackermann/steering_motor.h>
#include <vesc_ackermann/types.h>
#include <vesc_ackermann/wheel.h>

namespace vesc_ackermann
{
class Axle
{
public:
  Axle(const ros::NodeHandle& nh, const AckermannConfig& common_config, const AxleConfig& axle_config,
       std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory, double position_x);

  void setVelocity(double linear_velocity, double normalized_steering_angle, const ros::Time& time);

  void getVelocityConstraints(const ros::Time& time, VehicleVelocityConstraints& constraints);

  boost::optional<double> getSupplyVoltage();

protected:
  AckermannConfig common_config_;
  AxleConfig axle_config_;
  double position_x_;

  SteeringConstPtr steering_;
  Wheel left_wheel_;
  Wheel right_wheel_;

  boost::optional<SteeringMotor> steering_motor_;
  boost::optional<DriveMotor> left_motor_;
  boost::optional<DriveMotor> right_motor_;

  double last_steering_angle_ = 0.0;
  ros::Time last_steering_angle_time_;
};
}

#endif //VESC_ACKERMANN_AXLE_H
