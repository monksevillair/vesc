#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <vesc_driver/vesc_ros_driver.h>

namespace vesc_driver
{

class VescDriverNodelet: public nodelet::Nodelet
{
public:

  VescDriverNodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<VescRosDriver> vesc_driver_;

}; // class VescDriverNodelet

void VescDriverNodelet::onInit()
{
  NODELET_DEBUG("Initializing VESC driver nodelet");
  vesc_driver_.reset(new VescRosDriver(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace vesc_driver

PLUGINLIB_EXPORT_CLASS(vesc_driver::VescDriverNodelet, nodelet::Nodelet);
