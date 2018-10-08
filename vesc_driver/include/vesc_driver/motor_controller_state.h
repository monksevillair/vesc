/*
Created by clemens on 6/12/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_MOTOR_CONTROLLER_STATE_H
#define VESC_DRIVER_MOTOR_CONTROLLER_STATE_H

#include <cstdint>

namespace vesc_driver
{
struct MotorControllerState
{
  enum class FaultCode
  {
    NO_FAULT = 0, OVER_VOLTAGE, UNDER_VOLTAGE, DRV8302, ABS_OVER_CURRENT, OVER_TEMP_FET, OVER_TEMP_MOTOR
  };

  FaultCode fault_code = FaultCode::NO_FAULT;
  double voltage_input = 0.0;     // input voltage (V)
  double current_motor = 0.0;     // motor current (A)
  double current_input = 0.0;     // input current (A)
  double speed = 0.0;             // electrical speed (ERPM); real speed needs to consider the number of poles
  double position = 0.0;          // position (degrees)
  double duty_cycle = 0.0;        // duty cycle in [0, 1]
  double charge_drawn = 0.0;      // electric charge drawn (Ah)
  double charge_regen = 0.0;      // electrical charge regenerated (Ah)
  int32_t displacement = 0;       // net tachometer (counts)
  int32_t distance_traveled = 0;  // total tachometer (counts)
};
}

#endif //VESC_DRIVER_MOTOR_CONTROLLER_STATE_H
