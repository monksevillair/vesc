//
// Created by abuchegger on 13.07.18.
//
#include <arti_base_control/utils.h>
#include <angles/angles.h>

namespace arti_base_control
{

double normalizeSteeringAngle(const double steering_angle)
{
  const double normalized_angle = angles::normalize_angle(steering_angle);
  if (normalized_angle < -M_PI_2)
  {
    return normalized_angle + M_PI;
  }
  if (normalized_angle > M_PI_2)
  {
    return normalized_angle - M_PI;
  }
  return normalized_angle;
}

}
