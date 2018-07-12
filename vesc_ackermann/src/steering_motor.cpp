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
SteeringMotor::SteeringMotor(const MotorFactoryPtr& motor_factory, ros::NodeHandle& private_nh,
                             bool publish_motor_position)
  : motor_(motor_factory->createSteeringMotor(private_nh)),
    position_sent_publisher_(private_nh, "position_sent", publish_motor_position),
    position_received_publisher_(private_nh, "position_received", publish_motor_position)
{
}

double SteeringMotor::getPosition(const ros::Time& time)
{
  const double position = motor_->getPosition(time);
  position_received_publisher_.publish(position);

  if (time > last_position_time_)
  {
    if (!last_position_time_.isZero())
    {
      const double time_difference = (time - last_position_time_).toSec();
      if (time_difference < 1.0)
      {
        last_velocity_ = (position - last_position_) / time_difference;
      }
      else
      {
        last_velocity_ = 0.0;
      }
    }

    last_position_ = position;
    last_position_time_ = time;
  }

  return position;
}

double SteeringMotor::getVelocity(const ros::Time& time)
{
  if (time >= (last_position_time_ + ros::Duration(0.1)))
  {
    getPosition(time);
  }

  return last_velocity_;
}

void SteeringMotor::setPosition(double position)
{
  position_sent_publisher_.publish(position);
  motor_->setPosition(position);
}

double SteeringMotor::getSupplyVoltage()
{
  return motor_->getSupplyVoltage();
}
}
