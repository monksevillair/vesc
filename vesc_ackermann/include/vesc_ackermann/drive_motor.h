/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_DRIVE_MOTOR_H
#define VESC_ACKERMANN_DRIVE_MOTOR_H

#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <vesc_ackermann/optional_data_publisher.h>
#include <vesc_ackermann/types.h>
#include <vesc_motor/vesc_drive_motor.h>

namespace vesc_ackermann
{
class DriveMotor
{
public:
  DriveMotor(const MotorFactoryPtr& motor_factory, ros::NodeHandle& private_nh, bool publish_motor_speed);

  double getPosition(const ros::Time& time);
  double getVelocity(const ros::Time& time);

  void setVelocity(double velocity);

  void brake(double current);

  double getSupplyVoltage();

private:
  std::shared_ptr<vesc_motor::VescDriveMotor> motor_;
  OptionalDataPublisher<std_msgs::Float64> velocity_sent_publisher_;
  OptionalDataPublisher<std_msgs::Float64> velocity_received_publisher;
  double last_position_ = 0.0;
  double last_velocity_ = 0.0;
  ros::Time last_velocity_time_;
};
}

#endif //VESC_ACKERMANN_DRIVE_MOTOR_H
