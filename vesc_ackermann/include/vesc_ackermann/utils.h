//
// Created by abuchegger on 13.07.18.
//
#ifndef VESC_ACKERMANN_UTILS_H
#define VESC_ACKERMANN_UTILS_H

#include <ros/publisher.h>

namespace vesc_ackermann
{

double normalizeSteeringAngle(double steering_angle);

template<typename M>
void publishData(ros::Publisher& publisher, const typename M::_data_type& data)
{
  M msg;
  msg.data = data;
  publisher.publish(msg);
}

}

#endif //VESC_ACKERMANN_UTILS_H
