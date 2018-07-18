/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VESC_ACKERMANN_STEERING_MOTOR_H
#define VESC_ACKERMANN_STEERING_MOTOR_H

#include <boost/optional.hpp>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <vesc_ackermann/optional_data_publisher.h>
#include <vesc_ackermann/types.h>
#include <vesc_motor/vesc_steering_motor.h>

namespace vesc_ackermann
{
class SteeringMotor
{
public:
  virtual ~SteeringMotor() = default;

  virtual double getPosition(const ros::Time& time) = 0;
  virtual double getVelocity(const ros::Time& time) = 0;

  virtual void setPosition(double position) = 0;

  virtual boost::optional<double> getSupplyVoltage() = 0;
};

class VescSteeringMotor : public SteeringMotor
{
public:
  VescSteeringMotor(ros::NodeHandle& private_nh,
                    const std::shared_ptr<vesc_motor::VescTransportFactory>& transport_factory,
                    double control_interval);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  vesc_motor::VescSteeringMotor motor_;
};

class PublishingSteeringMotor : public SteeringMotor
{
public:
  PublishingSteeringMotor(ros::NodeHandle& private_nh, const SteeringMotorPtr& motor);

  double getPosition(const ros::Time& time) override;
  double getVelocity(const ros::Time& time) override;

  void setPosition(double position) override;

  boost::optional<double> getSupplyVoltage() override;

protected:
  SteeringMotorPtr motor_;
  OptionalDataPublisher<std_msgs::Float64> position_sent_publisher_;
  OptionalDataPublisher<std_msgs::Float64> position_received_publisher_;
  OptionalDataPublisher<std_msgs::Float64> velocity_received_publisher_;
};
}

#endif //VESC_ACKERMANN_STEERING_MOTOR_H
