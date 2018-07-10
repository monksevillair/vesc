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
DriveMotor::DriveMotor(const MotorFactoryPtr& motor_factory, ros::NodeHandle& private_nh, double control_interval,
                       bool publish_motor_speed)
  : motor_(motor_factory->createDriveMotor(private_nh, control_interval)),
    motor_speed_sent_(private_nh, "motor_speed_sent", publish_motor_speed),
    motor_speed_received_(private_nh, "motor_speed_received", publish_motor_speed)
{
}

double DriveMotor::getVelocity(const ros::Time& time)
{
  const double result = motor_->getVelocity(time);
  motor_speed_received_.publish(result);
  return result;
}

void DriveMotor::setVelocity(double velocity)
{
  motor_speed_sent_.publish(velocity);
  motor_->setVelocity(velocity);
}

void DriveMotor::brake(double current)
{
  motor_speed_sent_.publish(0.0);
  motor_->brake(current);
}

double DriveMotor::getSupplyVoltage()
{
  return motor_->getSupplyVoltage();
}
}
