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
    MotorControllerState() :
        fault_code(FAULT_CODE::NO_FAULT), voltage_input(0.), current_motor(0.), current_input(0.), speed(0.),
        position(0.), duty_cycle(0.), charge_drawn(0.), displacement(0), distance_traveled(0), charge_regen(0.)
    { }
    enum class FAULT_CODE{NO_FAULT, OVER_VOLTAGE, UNDER_VOLTAGE, DRV8302, ABS_OVER_CURRENT, OVER_TEMP_FET, OVER_TEMP_MOTOR};

    FAULT_CODE fault_code;
    double voltage_input;        // input voltage in (V)
    double current_motor;        // motor current (A)
    double current_input;        // input current (A)
    double speed;                // motor electrical speed (RRPM) real speed needs to consider the number of pools for the motor
    double position;             // motor position (rad)
    double duty_cycle;           // duty cycle [0, 1]
    double charge_drawn;         // electric charge drawn (Ah)
    double charge_regen;         // electrical charge regained (Ah)
    uint64_t displacement;       // net tachometer (counts)
    uint64_t distance_traveled;  // total tachometer (counts
  };
}

#endif //VESC_DRIVER_MOTOR_CONTROLLER_STATE_H
