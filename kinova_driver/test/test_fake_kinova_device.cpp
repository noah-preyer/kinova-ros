#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

#include "kinova_driver/conversions.hpp"
#include "kinova_driver/fake_kinova_device.hpp"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_device.hpp"

namespace
{

AngularInfo velocity(float actuator1)
{
  AngularInfo v{};
  v.Actuator1 = actuator1;
  return v;
}

kinova::FingerAngles fingers(float f1, float f2, float f3)
{
  kinova::FingerAngles f;
  f.Finger1 = f1;
  f.Finger2 = f2;
  f.Finger3 = f3;
  return f;
}

}  // namespace

// All timing tests drive the fake with a manual clock: double t = 0; auto clk = [&]{ return t; };

TEST(FakeKinovaDevice, IntegratesCommandedVelocity)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  t = 0.01;
  kinova::KinovaAngles angles;
  dev.getJointAngles(angles);

  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0] + 0.1, 1e-4);  // float storage
  EXPECT_NEAR(angles.Actuator2, kinova::kFakeHomeDeg[1], 1e-4);
}

TEST(FakeKinovaDevice, StopsMovingAfterCommandTimeout)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  t = 1.0;
  kinova::KinovaAngles angles;
  kinova::KinovaAngles velocities;
  dev.getJointAngles(angles);
  dev.getJointVelocities(velocities);

  // 10 deg/s for the 0.05 s the command stays in effect.
  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0] + 0.5, 1e-4);
  EXPECT_EQ(velocities.Actuator1, 0.0f);
}

TEST(FakeKinovaDevice, FingerCommandAppliesImmediately)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(0.0f), fingers(3400, 3400, 0));
  kinova::FingerAngles read;
  dev.getFingerPositions(read);

  EXPECT_EQ(read.Finger1, 3400.0f);
  EXPECT_EQ(read.Finger2, 3400.0f);
  EXPECT_EQ(dev.fingerCommandCount(), 1);
  EXPECT_EQ(dev.lastFingerCommand().Finger1, 3400.0f);
}

TEST(FakeKinovaDevice, IgnoresCommandsWhileStopped)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.stopAPI();
  EXPECT_TRUE(dev.isStopped());
  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(100, 100, 0));
  dev.setFingerPositions(fingers(200, 200, 0));
  t = 0.01;
  kinova::KinovaAngles angles;
  kinova::FingerAngles read;
  dev.getJointAngles(angles);
  dev.getFingerPositions(read);

  EXPECT_EQ(dev.velocityCommandCount(), 0);
  EXPECT_EQ(dev.fingerCommandCount(), 0);
  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0], 1e-4);
  EXPECT_EQ(read.Finger1, 0.0f);

  dev.startAPI();
  EXPECT_FALSE(dev.isStopped());
  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  EXPECT_EQ(dev.velocityCommandCount(), 1);
}

TEST(FakeKinovaDevice, HomeRestoresInitialPose)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointAnglesDeg({0, 0, 0, 0, 0, 0});
  kinova::KinovaAngles angles;
  dev.getJointAngles(angles);
  EXPECT_EQ(angles.Actuator3, 0.0f);

  dev.homeArm();
  dev.getJointAngles(angles);

  EXPECT_EQ(dev.homeCount(), 1);
  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0], 1e-4);
  EXPECT_NEAR(angles.Actuator2, kinova::kFakeHomeDeg[1], 1e-4);
  EXPECT_NEAR(angles.Actuator3, kinova::kFakeHomeDeg[2], 1e-4);
  EXPECT_NEAR(angles.Actuator4, kinova::kFakeHomeDeg[3], 1e-4);
  EXPECT_NEAR(angles.Actuator5, kinova::kFakeHomeDeg[4], 1e-4);
  EXPECT_NEAR(angles.Actuator6, kinova::kFakeHomeDeg[5], 1e-4);
}

TEST(FakeKinovaDevice, HomePoseIsTheSrdfHomeInDegrees)
{
  EXPECT_NEAR(kinova::kFakeHomeDeg[0], kinova_driver::radToDeg(4.8046852), 1e-12);
  EXPECT_NEAR(kinova::kFakeHomeDeg[1], kinova_driver::radToDeg(2.92482), 1e-12);
  EXPECT_NEAR(kinova::kFakeHomeDeg[2], kinova_driver::radToDeg(1.002), 1e-12);
  EXPECT_NEAR(kinova::kFakeHomeDeg[3], kinova_driver::radToDeg(4.2031852), 1e-12);
  EXPECT_NEAR(kinova::kFakeHomeDeg[4], kinova_driver::radToDeg(1.4458), 1e-12);
  EXPECT_NEAR(kinova::kFakeHomeDeg[5], kinova_driver::radToDeg(1.3233), 1e-12);
}

TEST(FakeKinovaDevice, ReportsCommandedVelocityWithinTimeout)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  t = 0.049;
  kinova::KinovaAngles velocities;
  dev.getJointVelocities(velocities);
  EXPECT_EQ(velocities.Actuator1, 10.0f);
  EXPECT_EQ(dev.lastVelocityCommand().Actuator1, 10.0f);

  t = 0.051;
  dev.getJointVelocities(velocities);
  EXPECT_EQ(velocities.Actuator1, 0.0f);
}

TEST(FakeKinovaDevice, NewCommandTakesEffectWhenReceived)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  t = 0.01;  // 0.1 deg at 10 deg/s
  dev.setJointVelocitiesWithFingerPositions(velocity(-20.0f), fingers(0, 0, 0));
  t = 0.02;  // -0.2 deg at -20 deg/s
  kinova::KinovaAngles angles;
  dev.getJointAngles(angles);

  EXPECT_EQ(dev.velocityCommandCount(), 2);
  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0] + 0.1 - 0.2, 1e-4);
}

TEST(FakeKinovaDevice, StopApiZeroesVelocityAndKeepsMotionUpToStop)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setJointVelocitiesWithFingerPositions(velocity(10.0f), fingers(0, 0, 0));
  t = 0.02;
  dev.stopAPI();
  t = 0.04;
  kinova::KinovaAngles angles;
  kinova::KinovaAngles velocities;
  dev.getJointAngles(angles);
  dev.getJointVelocities(velocities);

  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0] + 0.2, 1e-4);
  EXPECT_EQ(velocities.Actuator1, 0.0f);
}

TEST(FakeKinovaDevice, SetFingerPositionsCountsAndTorquesAreZero)
{
  double t = 0;
  auto clk = [&] {return t;};
  kinova::FakeKinovaDevice dev(kinova::kFakeHomeDeg, clk);

  dev.setFingerPositions(fingers(1000, 2000, 0), 0, true);
  dev.setFingerTurns(10.0, 20.0);
  kinova::FingerAngles read;
  dev.getFingerPositions(read);
  EXPECT_EQ(read.Finger1, 10.0f);
  EXPECT_EQ(read.Finger2, 20.0f);
  EXPECT_EQ(dev.fingerCommandCount(), 1);  // setFingerTurns is a test hook, not a command
  EXPECT_EQ(dev.lastFingerCommand().Finger2, 2000.0f);

  kinova::KinovaAngles torques;
  torques.Actuator1 = 5.0f;
  dev.getJointTorques(torques);
  EXPECT_EQ(torques.Actuator1, 0.0f);
  EXPECT_EQ(torques.Actuator6, 0.0f);
}

TEST(FakeKinovaDevice, DefaultClockUsesSteadyClockSeconds)
{
  kinova::FakeKinovaDevice dev;
  dev.setJointVelocitiesWithFingerPositions(velocity(0.0f), fingers(0, 0, 0));
  EXPECT_EQ(dev.velocityCommandCount(), 1);
  kinova::KinovaAngles angles;
  dev.getJointAngles(angles);
  EXPECT_NEAR(angles.Actuator1, kinova::kFakeHomeDeg[0], 1e-4);
}

TEST(KinovaDeviceSeam, CommAndFakeBothImplementTheInterface)
{
  static_assert(std::is_base_of<kinova::KinovaDevice, kinova::KinovaComm>::value, "KinovaComm");
  static_assert(std::is_base_of<kinova::KinovaDevice, kinova::FakeKinovaDevice>::value, "Fake");
  std::unique_ptr<kinova::KinovaDevice> dev = std::make_unique<kinova::FakeKinovaDevice>();
  EXPECT_FALSE(dev->isStopped());
}
