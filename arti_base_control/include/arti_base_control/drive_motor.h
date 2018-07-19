/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ARTI_BASE_CONTROL_DRIVE_MOTOR_H
#define ARTI_BASE_CONTROL_DRIVE_MOTOR_H

#include <arti_base_control/types.h>
#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <vesc_motor/types.h>
#include <vesc_motor/vesc_drive_motor.h>

namespace arti_base_control
{
class DriveMotor
{
public:
  virtual ~DriveMotor() = default;

  virtual double getPosition(const ros::Time& time) = 0;
  virtual double getVelocity(const ros::Time& time) = 0;

  virtual void setVelocity(double velocity) = 0;

  virtual void brake(double current) = 0;

  virtual boost::optional<double> getSupplyVoltage() = 0;
};

class VescDriveMotor : public DriveMotor
{
public:
  VescDriveMotor(ros::NodeHandle& private_nh, const vesc_motor::DriverFactoryPtr& driver_factory,
                 double control_interval);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  vesc_motor::VescDriveMotor motor_;
  double last_position_ = 0.0;
  double last_velocity_ = 0.0;
  ros::Time last_velocity_time_;
};

class PublishingDriveMotor : public DriveMotor
{
public:
  PublishingDriveMotor(ros::NodeHandle& private_nh, const DriveMotorPtr& motor);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setVelocity(double velocity) override;

  void brake(double current) override;

  boost::optional<double> getSupplyVoltage() override;

private:
  DriveMotorPtr motor_;
  ros::Publisher velocity_sent_publisher_;
  ros::Publisher velocity_received_publisher_;
  ros::Publisher position_received_publisher_;
};
}

#endif //ARTI_BASE_CONTROL_DRIVE_MOTOR_H
