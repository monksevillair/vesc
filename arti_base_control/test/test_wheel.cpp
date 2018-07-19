//
// Created by abuchegger on 08.07.18.
//
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <arti_base_control/steering.h>
#include <arti_base_control/wheel.h>

TEST(WheelTest, SimpleFrontWheelTest)
{
  const arti_base_control::Wheel wheel(1.0, 0.0, 0.0, 0.5, std::make_shared<arti_base_control::IdealAckermannSteering>(0.0));
  EXPECT_DOUBLE_EQ(2.0, wheel.computeWheelVelocity(1.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, 1.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(1.0, -1.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, 1.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(-2.0 * std::sqrt(2.0), wheel.computeWheelVelocity(-1.0, -1.0, 0.0, 0.0));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
