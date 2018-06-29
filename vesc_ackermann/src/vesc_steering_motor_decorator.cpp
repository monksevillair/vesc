/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_ackermann/vesc_steering_motor_decorator.h>

namespace vesc_ackermann
{

  VescSteeringMotorDecorator::VescSteeringMotorDecorator(const ros::NodeHandle &private_nh,
                                                         std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                                                         double execution_duration, bool publish_motor_position) :
      motor_(private_nh, transport_factory, execution_duration),
      motor_position_send_("position_send", private_nh, publish_motor_position),
      motor_position_reveiced_("position_reveiced", private_nh, publish_motor_position)
  { }

  double VescSteeringMotorDecorator::getPosition(const ros::Time &time)
  {
    double result = motor_.getPosition(time);
    motor_position_reveiced_.publishData(result);

    return result;
  }

  void VescSteeringMotorDecorator::setPosition(double position)
  {
    motor_position_send_.publishData(position);
    motor_.setPosition(position);
  }

  double VescSteeringMotorDecorator::getSupplyVoltage()
  {
    return motor_.getSupplyVoltage();
  }
}
