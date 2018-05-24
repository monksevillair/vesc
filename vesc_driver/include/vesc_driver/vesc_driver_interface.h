/*
Created by clemens on 5/16/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_INTERFACE_H
#define VESC_DRIVER_VESC_DRIVER_INTERFACE_H

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

namespace vesc_driver
{
class VescDriverInterface
{
public:

  typedef boost::function<void(const std_msgs::Float64::Ptr&)> ServoSensorHandlerFunction;
  typedef boost::function<void(const vesc_msgs::VescStateStamped::Ptr&)> StateHandlerFunction;

  explicit VescDriverInterface(const ServoSensorHandlerFunction& servo_sensor_handler = ServoSensorHandlerFunction(),
                               const StateHandlerFunction& state_handler = StateHandlerFunction())
    : servo_sensor_handler_(servo_sensor_handler), state_handler_(state_handler)
  {}

  /**
   * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
   *                   note that the VESC may impose a more restrictive bounds on the range depending
   *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
   */
  virtual void setDutyCycle(const std_msgs::Float64::ConstPtr& duty_cycle) = 0;
  /**
   * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
   *                note that the VESC may impose a more restrictive bounds on the range depending on
   *                its configuration.
   */
  virtual void setCurrent(const std_msgs::Float64::ConstPtr& current) = 0;
  /**
   * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
   *              However, note that the VESC may impose a more restrictive bounds on the range
   *              depending on its configuration.
   */
  virtual void setBrake(const std_msgs::Float64::ConstPtr& brake) = 0;

  /**
   * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
   *              multiplied by the number of motor poles. Any value is accepted by this
   *              driver. However, note that the VESC may impose a more restrictive bounds on the
   *              range depending on its configuration.
   */
  virtual void setSpeed(const std_msgs::Float64::ConstPtr& speed) = 0;
  /**
   * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
   *                 Note that the VESC must be in encoder mode for this command to have an effect.
   */
  virtual void setPosition(const std_msgs::Float64::ConstPtr& position) = 0;
  /**
   * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
   */
  virtual void setServo(const std_msgs::Float64::ConstPtr& servo) = 0;

  virtual bool executionCycle() = 0;

protected:
  ServoSensorHandlerFunction servo_sensor_handler_;
  StateHandlerFunction state_handler_;
};

} // namespace vesc_driver

#endif //VESC_DRIVER_VESC_DRIVER_INTERFACE_H
