//
// Created by abuchegger on 08.07.18.
//
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vesc_ackermann/steering.h>
#include <vesc_ackermann/wheel.h>

TEST(WheelTest, SimpleFrontWheelTest)
{
  const vesc_ackermann::Wheel wheel(1.0, 0.0, 0.0, 0.5, std::make_shared<vesc_ackermann::IdealAckermannSteering>(0.0));
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
