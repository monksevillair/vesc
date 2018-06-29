/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_INTERFACE_H
#define VESC_DRIVER_VESC_DRIVER_INTERFACE_H

#include <vesc_driver/motor_controller_state.h>
#include <functional>

namespace vesc_driver
{
class VescDriverInterface
{
public:
  typedef std::function<void(const MotorControllerState&)> StateHandlerFunction;

  explicit VescDriverInterface(const StateHandlerFunction& state_handler_function)
    : state_handler_function_(state_handler_function)
  {}

  /**
   * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
   *                   note that the VESC may impose a more restrictive bounds on the range depending
   *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
   */
  virtual void setDutyCycle(double duty_cycle) = 0;

  /**
   * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
   *                note that the VESC may impose a more restrictive bounds on the range depending on
   *                its configuration.
   */
  virtual void setCurrent(double current) = 0;

  /**
   * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
   *              However, note that the VESC may impose a more restrictive bounds on the range
   *              depending on its configuration.
   */
  virtual void setBrake(double brake_current) = 0;

  /**
   * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
   *              multiplied by the number of motor poles. Any value is accepted by this
   *              driver. However, note that the VESC may impose a more restrictive bounds on the
   *              range depending on its configuration.
   */
  virtual void setSpeed(double speed) = 0;

  /**
   * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
   *                 Note that the VESC must be in encoder mode for this command to have an effect.
   */
  virtual void setPosition(double position) = 0;

protected:
  StateHandlerFunction state_handler_function_;
};
}

#endif //VESC_DRIVER_VESC_DRIVER_INTERFACE_H
