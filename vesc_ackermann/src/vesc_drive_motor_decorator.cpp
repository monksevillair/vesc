/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_ackermann/vesc_drive_motor_decorator.h>

namespace vesc_ackermann
{

  VescDriveMotorDecorator::VescDriveMotorDecorator(const ros::NodeHandle &private_nh,
                                                   std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                                                   double execution_duration, bool publish_motor_speed) :
      motor_(private_nh, transport_factory, execution_duration),
      motor_speed_send_("motor_speed_send", private_nh, publish_motor_speed),
      motor_speed_reveiced_("motor_speed_received", private_nh, publish_motor_speed)
  { }

  double VescDriveMotorDecorator::getVelocity(const ros::Time &time)
  {
    double result = motor_.getVelocity(time);
    motor_speed_reveiced_.publishData(result);

    return result;
  }

  void VescDriveMotorDecorator::setVelocity(double velocity)
  {
    motor_speed_send_.publishData(velocity);
    motor_.setVelocity(velocity);
  }

  void VescDriveMotorDecorator::brake(double current)
  {
    motor_speed_send_.publishData(0.);
    motor_.brake(current);
  }

  double VescDriveMotorDecorator::getSupplyVoltage()
  {
    return motor_.getSupplyVoltage();
  }
}
