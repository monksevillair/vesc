/*
Created by clemens on 06.02.18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_VESCROSDRIVER_H
#define VESC_DRIVER_VESCROSDRIVER_H

#include <vesc_driver/vesc_driver.h>

namespace vesc_driver
{
class VescRosDriver
{
public:
  VescRosDriver(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  VescDriver driver_;

  // ROS publisher and subscriber
  ros::Publisher state_pub_;
  ros::Publisher servo_sensor_pub_;

  ros::Subscriber duty_cycle_sub_;
  ros::Subscriber current_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber servo_sub_;

  ros::Timer timer_;

  /**
   * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
   *                   note that the VESC may impose a more restrictive bounds on the range depending
   *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
   */
  void dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle);
  /**
   * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
   *                note that the VESC may impose a more restrictive bounds on the range depending on
   *                its configuration.
   */
  void currentCallback(const std_msgs::Float64::ConstPtr& current);
  /**
   * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
   *              However, note that the VESC may impose a more restrictive bounds on the range
   *              depending on its configuration.
   */
  void brakeCallback(const std_msgs::Float64::ConstPtr& brake);

  /**
   * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
   *              multiplied by the number of motor poles. Any value is accepted by this
   *              driver. However, note that the VESC may impose a more restrictive bounds on the
   *              range depending on its configuration.
   */
  void speedCallback(const std_msgs::Float64::ConstPtr& speed);
  /**
   * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
   *                 Note that the VESC must be in encoder mode for this command to have an effect.
   */
  void positionCallback(const std_msgs::Float64::ConstPtr& position);
  /**
   * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
   */
  void servoCallback(const std_msgs::Float64::ConstPtr& servo);

  void timerCallback(const ros::TimerEvent& event);

  void servoSensorCB(const boost::shared_ptr<std_msgs::Float64>& servo_sensor_value);

  void stateCB(const boost::shared_ptr<vesc_msgs::VescStateStamped>& state);

};
}

#endif //VESC_DRIVER_VESCROSDRIVER_H
