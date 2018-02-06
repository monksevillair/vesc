#include <ros/ros.h>

#include <vesc_driver/vesc_ros_driver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  vesc_driver::VescRosDriver vesc_driver(nh, private_nh);

  ros::spin();

  return 0;
}
