/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_ackermann/steering_motor.h>
#include <vesc_ackermann/motor_factory.h>

namespace vesc_ackermann
{
VescSteeringMotor::VescSteeringMotor(ros::NodeHandle& private_nh,
                                     const std::shared_ptr<vesc_motor::VescTransportFactory>& transport_factory,
                                     double control_interval)
  : motor_(private_nh, transport_factory, control_interval)
{
}

double VescSteeringMotor::getPosition(const ros::Time& time)
{
  return motor_.getPosition(time);
}

double VescSteeringMotor::getVelocity(const ros::Time& time)
{
  return motor_.getVelocity(time);
}

void VescSteeringMotor::setPosition(double position)
{
  motor_.setPosition(position);
}

boost::optional<double> VescSteeringMotor::getSupplyVoltage()
{
  return motor_.getSupplyVoltage();
}

PublishingSteeringMotor::PublishingSteeringMotor(ros::NodeHandle& private_nh, const SteeringMotorPtr& motor)
  : motor_(motor), position_sent_publisher_(private_nh, "position_sent", true),
    position_received_publisher_(private_nh, "position_received", true),
    velocity_received_publisher_(private_nh, "velocity_received", true)
{
}

double PublishingSteeringMotor::getPosition(const ros::Time& time)
{
  const double position = motor_->getPosition(time);
  position_received_publisher_.publish(position);
  return position;
}

double PublishingSteeringMotor::getVelocity(const ros::Time& time)
{
  const double velocity = motor_->getVelocity(time);
  velocity_received_publisher_.publish(velocity);
  return velocity;
}

void PublishingSteeringMotor::setPosition(double position)
{
  motor_->setPosition(position);
  position_sent_publisher_.publish(position);
}

boost::optional<double> PublishingSteeringMotor::getSupplyVoltage()
{
  return motor_->getSupplyVoltage();
}
}
