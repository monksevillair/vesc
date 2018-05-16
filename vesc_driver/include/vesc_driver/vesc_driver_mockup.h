/*
Created by clemens on 5/16/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESC_DRIVER_MOCKUP_H
#define VESC_DRIVER_VESC_DRIVER_MOCKUP_H

#include <vesc_driver/vesc_driver_interface.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace vesc_driver
{
  class VescDriverMockup : public VescDriverInterface
  {
    VescDriverMockup(const StateHandlerFunction& state_handler = StateHandlerFunction());

    /**
     * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
     *                   note that the VESC may impose a more restrictive bounds on the range depending
     *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
     */
    void setDutyCycle(const std_msgs::Float64::ConstPtr& duty_cycle) override;
    /**
     * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
     *                note that the VESC may impose a more restrictive bounds on the range depending on
     *                its configuration.
     */
    void setCurrent(const std_msgs::Float64::ConstPtr& current) override;
    /**
     * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
     *              However, note that the VESC may impose a more restrictive bounds on the range
     *              depending on its configuration.
     */
    void setBrake(const std_msgs::Float64::ConstPtr& brake) override;

    /**
     * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
     *              multiplied by the number of motor poles. Any value is accepted by this
     *              driver. However, note that the VESC may impose a more restrictive bounds on the
     *              range depending on its configuration.
     */
    void setSpeed(const std_msgs::Float64::ConstPtr& speed) override;
    /**
     * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
     *                 Note that the VESC must be in encoder mode for this command to have an effect.
     */
    void setPosition(const std_msgs::Float64::ConstPtr& position) override;
    /**
     * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
     */
    void setServo(const std_msgs::Float64::ConstPtr& servo) override;

    bool executionCycle() override;

  private:
    void run();

    boost::mutex state_mutex_;
    vesc_msgs::VescStateStamped current_state_;

    boost::thread state_thread_;
    bool send_state_;
    boost::condition_variable send_state_condition_;
  };
}

#endif //VESC_DRIVER_VESC_DRIVER_MOCKUP_H
