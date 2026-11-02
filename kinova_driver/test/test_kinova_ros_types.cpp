#include <gtest/gtest.h>

#include "kinova_driver/kinova_ros_types.h"

TEST(KinovaRosTypes, ValidatesRobotType) {
  EXPECT_TRUE(kinova::valid_kinovaRobotType("j2n6s200"));
  EXPECT_FALSE(kinova::valid_kinovaRobotType("j2n6s20"));
  EXPECT_FALSE(kinova::valid_kinovaRobotType("x2n6s200"));
}

TEST(KinovaRosTypes, AnglesMessageRoundTrip) {
  kinova_msgs::msg::JointAngles msg;
  msg.joint1 = 10.f;
  msg.joint6 = 350.f;
  kinova::KinovaAngles angles(msg);
  EXPECT_FLOAT_EQ(angles.Actuator1, 10.f);
  EXPECT_FLOAT_EQ(angles.constructAnglesMsg().joint6, 350.f);
}

TEST(KinovaRosTypes, PoseMessageUsesLowercaseFields) {
  CartesianInfo info{};
  info.X = 0.1f;
  info.ThetaZ = 0.5f;
  auto msg = kinova::KinovaPose(info).constructKinovaPoseMsg();
  EXPECT_FLOAT_EQ(msg.x, 0.1f);
  EXPECT_FLOAT_EQ(msg.theta_z, 0.5f);
}
