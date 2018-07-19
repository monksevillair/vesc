//
// Created by abuchegger on 13.07.18.
//
#ifndef ARTI_BASE_CONTROL_UTILS_H
#define ARTI_BASE_CONTROL_UTILS_H

#include <ros/publisher.h>

namespace arti_base_control
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

#endif //ARTI_BASE_CONTROL_UTILS_H
