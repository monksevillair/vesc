/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_ackermann/drive_motor.h>
#include <vesc_ackermann/motor_factory.h>

namespace vesc_ackermann
{
DriveMotor::DriveMotor(const MotorFactoryPtr& motor_factory, ros::NodeHandle& private_nh, bool publish_motor_speed)
  : motor_(motor_factory->createDriveMotor(private_nh)),
    velocity_sent_publisher_(private_nh, "velocity_sent", publish_motor_speed),
    velocity_received_publisher(private_nh, "velocity_received", publish_motor_speed)
{
}

double DriveMotor::getPosition(const ros::Time& time)
{
  if (time > last_velocity_time_)
  {
    getVelocity(time);
  }

  return last_position_;
}

double DriveMotor::getVelocity(const ros::Time& time)
{
  const double velocity = motor_->getVelocity(time);
  velocity_received_publisher.publish(velocity);

  if (time > last_velocity_time_)
  {
    if (!last_velocity_time_.isZero())
    {
      const double time_difference = (time - last_velocity_time_).toSec();
      last_position_ += (last_velocity_ + velocity) * 0.5 * time_difference;  // Assumes constant acceleration
    }

    last_velocity_ = velocity;
    last_velocity_time_ = time;
  }

  return velocity;
}

void DriveMotor::setVelocity(const double velocity)
{
  velocity_sent_publisher_.publish(velocity);
  motor_->setVelocity(velocity);
}

void DriveMotor::brake(const double current)
{
  velocity_sent_publisher_.publish(0.0);
  motor_->brake(current);
}

double DriveMotor::getSupplyVoltage()
{
  return motor_->getSupplyVoltage();
}
}
