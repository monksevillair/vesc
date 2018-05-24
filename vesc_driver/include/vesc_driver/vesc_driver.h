// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER_VESC_DRIVER_H_
#define VESC_DRIVER_VESC_DRIVER_H_

#include <boost/optional.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <string>
#include <vesc_driver/vesc_driver_interface.h>
#include <vesc_driver/vesc_interface.h>
#include <vesc_driver/vesc_packet.h>
#include <vesc_msgs/VescStateStamped.h>

namespace vesc_driver
{

class VescDriver : public VescDriverInterface
{
public:

  explicit VescDriver(ros::NodeHandle private_nh,
                      const ServoSensorHandlerFunction& servo_sensor_handler = ServoSensorHandlerFunction(),
                      const StateHandlerFunction& state_handler = StateHandlerFunction());

  /**
   * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
   *                   note that the VESC may impose a more restrictive bounds on the range depending
   *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
   */
  virtual void setDutyCycle(const std_msgs::Float64::ConstPtr& duty_cycle);
  /**
   * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
   *                note that the VESC may impose a more restrictive bounds on the range depending on
   *                its configuration.
   */
  virtual void setCurrent(const std_msgs::Float64::ConstPtr& current);
  /**
   * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
   *              However, note that the VESC may impose a more restrictive bounds on the range
   *              depending on its configuration.
   */
  virtual void setBrake(const std_msgs::Float64::ConstPtr& brake);

  /**
   * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
   *              multiplied by the number of motor poles. Any value is accepted by this
   *              driver. However, note that the VESC may impose a more restrictive bounds on the
   *              range depending on its configuration.
   */
  virtual void setSpeed(const std_msgs::Float64::ConstPtr& speed);
  /**
   * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
   *                 Note that the VESC must be in encoder mode for this command to have an effect.
   */
  virtual void setPosition(const std_msgs::Float64::ConstPtr& position);
  /**
   * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
   */
  virtual void setServo(const std_msgs::Float64::ConstPtr& servo);

  virtual bool executionCycle();

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet);
  void vescErrorCallback(const std::string& error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                 const boost::optional<double>& min_lower = boost::optional<double>(),
                 const boost::optional<double>& max_upper = boost::optional<double>());
    double clip(double value);
    std::string name;
    boost::optional<double> lower;
    boost::optional<double> upper;
  };
  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING
  } driver_mode_t;

  boost::mutex mode_mutex_;
  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc

  bool isInOperationMode();
};

} // namespace vesc_driver

#endif // VESC_DRIVER_VESC_DRIVER_H_
