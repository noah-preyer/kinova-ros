#include <gtest/gtest.h>

#include "kinova_system_test_fixture.hpp"

TEST(KinovaSystemInit, RejectsWrongJointCount)
{
  TestableKinovaSystem sys;
  EXPECT_EQ(sys.on_init(parse(makeTestUrdf("", /*drop_last_tip=*/true))), CallbackReturn::ERROR);
}

TEST(KinovaSystemInit, RejectsArmJointWithoutVelocityCommand)
{
  std::string urdf = makeTestUrdf();
  const auto joint_3 = urdf.find("<joint name=\"j2n6s200_joint_3\">");
  ASSERT_NE(joint_3, std::string::npos);
  const std::string velocity = "<command_interface name=\"velocity\">";
  const auto command = urdf.find(velocity, joint_3);
  ASSERT_NE(command, std::string::npos);
  urdf.replace(command, velocity.size(), "<command_interface name=\"position\">");

  TestableKinovaSystem sys;
  EXPECT_EQ(sys.on_init(parse(urdf)), CallbackReturn::ERROR);
}

TEST(KinovaSystemConfigure, FailsWhenDeviceThrows)
{
  TestableKinovaSystem sys;
  sys.throw_on_create = true;
  rclcpp_lifecycle::State lifecycle_state;
  ASSERT_EQ(sys.on_init(parse(makeTestUrdf())), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.on_configure(lifecycle_state), CallbackReturn::ERROR);
}

TEST_F(KinovaSystemTest, ExportsExpectedInterfaces)
{
  ASSERT_NO_FATAL_FAILURE(configure());

  std::set<std::string> expected_states;
  for (int n = 1; n <= 6; ++n) {
    const std::string joint = "j2n6s200_joint_" + std::to_string(n);
    expected_states.insert({joint + "/position", joint + "/velocity", joint + "/effort"});
  }
  for (int k = 1; k <= 2; ++k) {
    const std::string finger = "j2n6s200_joint_finger_" + std::to_string(k);
    expected_states.insert({finger + "/position", finger + "/velocity"});
    expected_states.insert("j2n6s200_joint_finger_tip_" + std::to_string(k) + "/position");
  }
  std::set<std::string> expected_commands;
  for (const auto & name : armVelocities()) {
    expected_commands.insert(name);
  }
  expected_commands.insert(kFingerCommand);

  EXPECT_EQ(state_names.size(), 24u);
  EXPECT_EQ(std::set<std::string>(state_names.begin(), state_names.end()), expected_states);
  EXPECT_EQ(command_names.size(), 7u);
  EXPECT_EQ(
    std::set<std::string>(command_names.begin(), command_names.end()), expected_commands);
}

TEST_F(KinovaSystemTest, ReadConvertsUnits)
{
  ASSERT_NO_FATAL_FAILURE(configure({90, 180, 90, 180, 90, 180}));
  sys.fake->setFingerTurns(6800, 6800);

  ASSERT_EQ(read(), return_type::OK);

  EXPECT_NEAR(state("j2n6s200_joint_1/position"), M_PI / 2, 1e-6);
  EXPECT_NEAR(state("j2n6s200_joint_2/position"), M_PI, 1e-6);
  EXPECT_NEAR(state("j2n6s200_joint_finger_1/position"), 1.396263, 1e-6);
  EXPECT_EQ(state("j2n6s200_joint_finger_tip_1/position"), 0.0);
}

TEST_F(KinovaSystemTest, ReadUnwrapsPositionJump)
{
  ASSERT_NO_FATAL_FAILURE(configure({359, 180, 90, 180, 90, 180}));
  ASSERT_EQ(read(), return_type::OK);

  sys.fake->setJointAnglesDeg({1, 180, 90, 180, 90, 180});
  ASSERT_EQ(read(), return_type::OK);

  EXPECT_NEAR(state("j2n6s200_joint_1/position"), 6.300639, 1e-6);
}

TEST_F(KinovaSystemTest, ReadFingerVelocityIsFiniteDifference)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_EQ(read(), return_type::OK);

  sys.fake->setFingerTurns(68, 136);  // 0.8 and 1.6 degrees
  ASSERT_EQ(read(0.01), return_type::OK);
  EXPECT_NEAR(state("j2n6s200_joint_finger_1/velocity"), kinova_driver::degToRad(80.0), 1e-4);
  EXPECT_NEAR(state("j2n6s200_joint_finger_2/velocity"), kinova_driver::degToRad(160.0), 1e-4);

  sys.fake->setFingerTurns(136, 136);
  ASSERT_EQ(read(0.0), return_type::OK);
  EXPECT_EQ(state("j2n6s200_joint_finger_1/velocity"), 0.0);
}

TEST_F(KinovaSystemTest, WriteSendsNothingWhenArmNotClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  setCommand(armVelocity(1), 0.1);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->velocityCommandCount(), 0);
  EXPECT_EQ(sys.fake->fingerCommandCount(), 0);
}

TEST_F(KinovaSystemTest, WriteSendsDegreesWhenClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), 0.1);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_NEAR(sys.fake->lastVelocityCommand().Actuator1, 5.729578, 1e-5);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 1);
}

TEST_F(KinovaSystemTest, WriteClampsAndSanitizesVelocity)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), std::numeric_limits<double>::quiet_NaN());
  setCommand(armVelocity(2), 10.0);
  setCommand(armVelocity(4), -10.0);

  ASSERT_EQ(write(), return_type::OK);

  const AngularInfo sent = sys.fake->lastVelocityCommand();
  EXPECT_EQ(sent.Actuator1, 0.0f);
  EXPECT_NEAR(sent.Actuator2, 36.0, 1e-3);
  EXPECT_NEAR(sent.Actuator4, -48.0, 1e-3);
}

TEST_F(KinovaSystemTest, StopSwitchZeroesArmVelocity)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);
  ASSERT_EQ(sys.fake->velocityCommandCount(), 1);

  ASSERT_NO_FATAL_FAILURE(release(armVelocities()));
  EXPECT_EQ(commands.at(armVelocity(1)).get_value(), 0.0);
  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->velocityCommandCount(), 2);
  const AngularInfo sent = sys.fake->lastVelocityCommand();
  EXPECT_EQ(sent.Actuator1, 0.0f);
  EXPECT_EQ(sent.Actuator2, 0.0f);
  EXPECT_EQ(sent.Actuator3, 0.0f);
  EXPECT_EQ(sent.Actuator4, 0.0f);
  EXPECT_EQ(sent.Actuator5, 0.0f);
  EXPECT_EQ(sent.Actuator6, 0.0f);

  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 2);
}

TEST_F(KinovaSystemTest, RejectsPartialArmClaim)
{
  ASSERT_NO_FATAL_FAILURE(configure());

  EXPECT_EQ(switchInterfaces(armVelocities(1, 5), {}), return_type::ERROR);

  EXPECT_FALSE(sys.armVelocityClaimed());
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 0);
}

TEST_F(KinovaSystemTest, StopAndStartInOneSwitchKeepsArmClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));

  ASSERT_EQ(switchInterfaces(armVelocities(), armVelocities()), return_type::OK);

  EXPECT_TRUE(sys.armVelocityClaimed());
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 1);
  EXPECT_NEAR(sys.fake->lastVelocityCommand().Actuator1, 5.729578, 1e-5);
}

TEST_F(KinovaSystemTest, FingerCommandEmbeddedWhileArmClaimed)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  std::vector<std::string> all = armVelocities();
  all.push_back(kFingerCommand);
  ASSERT_NO_FATAL_FAILURE(claim(all));
  setCommand(kFingerCommand, 0.698132);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->velocityCommandCount(), 1);
  EXPECT_NEAR(sys.fake->lastFingerCommand().Finger1, 3400.0, 0.5);
  EXPECT_NEAR(sys.fake->lastFingerCommand().Finger2, 3400.0, 0.5);
}

TEST_F(KinovaSystemTest, FingerCommandSentAloneWhenArmIdle)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim({kFingerCommand}));
  setCommand(kFingerCommand, 0.698132);

  ASSERT_EQ(write(), return_type::OK);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->fingerCommandCount(), 1);

  setCommand(kFingerCommand, 0.8);
  ASSERT_EQ(write(), return_type::OK);
  EXPECT_EQ(sys.fake->fingerCommandCount(), 2);
  EXPECT_EQ(sys.fake->velocityCommandCount(), 0);
}

TEST_F(KinovaSystemTest, FingerCommandClampedToMaxTurn)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim({kFingerCommand}));
  setCommand(kFingerCommand, 1.51);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->lastFingerCommand().Finger1, 6800.0f);
}

TEST_F(KinovaSystemTest, WriteSilentWhileStopped)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  sys.fake->stopAPI();
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), 0.1);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->velocityCommandCount(), 0);
  EXPECT_EQ(sys.fake->rawCalls(), 0);
}

TEST_F(KinovaSystemTest, WriteSilentWhileInactive)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), 0.1);

  ASSERT_EQ(sys.on_deactivate(lifecycle_state), CallbackReturn::SUCCESS);
  ASSERT_EQ(sys.fake->raw_velocity_calls, 1);
  EXPECT_EQ(sys.fake->raw_finger_calls, 0);
  EXPECT_EQ(sys.fake->last_raw_velocity.Actuator1, 0.0f);

  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->raw_velocity_calls, 1);
  EXPECT_EQ(sys.fake->raw_finger_calls, 0);
}

TEST_F(KinovaSystemTest, WriteSilentAfterErrorAndReconfigure)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  ASSERT_NO_FATAL_FAILURE(claim(armVelocities()));
  setCommand(armVelocity(1), 0.1);

  ASSERT_EQ(sys.on_error(lifecycle_state), CallbackReturn::SUCCESS);
  EXPECT_EQ(sys.fake->raw_velocity_calls, 1);  // best-effort zero point
  EXPECT_EQ(sys.fake->last_raw_velocity.Actuator1, 0.0f);
  EXPECT_EQ(commands.at(armVelocity(1)).get_value(), 0.0);

  // The arm is still claimed, but the component is only INACTIVE after on_configure.
  ASSERT_EQ(sys.on_configure(lifecycle_state), CallbackReturn::SUCCESS);  // new fake device
  setCommand(armVelocity(1), 0.1);
  ASSERT_EQ(write(), return_type::OK);

  EXPECT_EQ(sys.fake->rawCalls(), 0);
}

TEST_F(KinovaSystemTest, DeactivateSendsOneZeroPoint)
{
  ASSERT_NO_FATAL_FAILURE(configure());
  sys.fake->setFingerTurns(3400, 3400);
  ASSERT_EQ(sys.on_activate(lifecycle_state), CallbackReturn::SUCCESS);

  ASSERT_EQ(sys.on_deactivate(lifecycle_state), CallbackReturn::SUCCESS);

  EXPECT_EQ(sys.fake->velocityCommandCount(), 1);
  EXPECT_EQ(sys.fake->lastVelocityCommand().Actuator1, 0.0f);
  EXPECT_EQ(sys.fake->lastFingerCommand().Finger1, 3400.0f);
}
