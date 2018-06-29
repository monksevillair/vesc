/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_ACKERMANN_VESC_STEERING_MOTOR_DECORATOR_H
#define VESC_ACKERMANN_VESC_STEERING_MOTOR_DECORATOR_H

#include <vesc_motor/vesc_steering_motor.h>
#include <vesc_motor/vesc_transport_factory.h>
#include <vesc_ackermann/publish_helper.h>

namespace vesc_ackermann
{
  class VescSteeringMotorDecorator
  {
  public:
    VescSteeringMotorDecorator(const ros::NodeHandle& private_nh,
                               std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory,
                               double execution_duration, bool publish_motor_position);

    double getPosition(const ros::Time& time);

    void setPosition(double position);

    double getSupplyVoltage();

  private:
    vesc_motor::VescSteeringMotor motor_;
    PublishHelper motor_position_send_;
    PublishHelper motor_position_reveiced_;
  };
}

#endif //VESC_ACKERMANN_VESC_STEERING_MOTOR_DECORATOR_H
