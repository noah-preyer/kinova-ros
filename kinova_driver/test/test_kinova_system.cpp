#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "kinova_driver/conversions.hpp"
#include "kinova_driver/fake_kinova_device.hpp"
#include "kinova_driver/kinova_ros_types.h"
#include "kinova_driver/kinova_system.hpp"
#include "kinova_test_urdf.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

const char kFingerCommand[] = "j2n6s200_joint_finger_1/position";

std::string armVelocity(int n)
{
  return "j2n6s200_joint_" + std::to_string(n) + "/velocity";
}

std::vector<std::string> armVelocities(int first = 1, int last = 6)
{
  std::vector<std::string> names;
  for (int n = first; n <= last; ++n) {
    names.push_back(armVelocity(n));
  }
  return names;
}

hardware_interface::HardwareInfo parse(const std::string & urdf)
{
  return hardware_interface::parse_control_resources_from_urdf(urdf)[0];
}

/// FakeKinovaDevice that also counts every set* call that reaches it, including the calls the
/// fake ignores (and does not count) while the API is stopped.
class CountingFakeKinovaDevice : public kinova::FakeKinovaDevice
{
public:
  using kinova::FakeKinovaDevice::FakeKinovaDevice;

  void setJointVelocitiesWithFingerPositions(
    const AngularInfo & joint_vel_deg_s, const kinova::FingerAngles & fingers_turns) override
  {
    ++raw_velocity_calls;
    last_raw_velocity = joint_vel_deg_s;
    kinova::FakeKinovaDevice::setJointVelocitiesWithFingerPositions(joint_vel_deg_s, fingers_turns);
  }

  void setFingerPositions(
    const kinova::FingerAngles & fingers, int timeout = 0, bool push = true) override
  {
    ++raw_finger_calls;
    kinova::FakeKinovaDevice::setFingerPositions(fingers, timeout, push);
  }

  int rawCalls() const {return raw_velocity_calls + raw_finger_calls;}

  int raw_velocity_calls = 0;
  int raw_finger_calls = 0;
  AngularInfo last_raw_velocity{};
};

/// KinovaSystem on a CountingFakeKinovaDevice that runs on a manual clock (t, in seconds).
class TestableKinovaSystem : public kinova_driver::KinovaSystem
{
public:
  std::array<double, 6> initial_deg = kinova::kFakeHomeDeg;
  bool throw_on_create = false;
  double t = 0.0;
  CountingFakeKinovaDevice * fake = nullptr;

  bool armVelocityClaimed() const {return arm_velocity_claimed_;}

protected:
  std::unique_ptr<kinova::KinovaDevice> createDevice() override
  {
    if (throw_on_create) {
      throw kinova::KinovaCommException("no arm", 1);
    }
    auto device = std::make_unique<CountingFakeKinovaDevice>(initial_deg, [this] {return t;});
    fake = device.get();
    return device;
  }
};

class KinovaSystemTest : public ::testing::Test
{
protected:
  /// on_init -> on_configure -> export -> on_activate, with the fake starting at initial_deg.
  void configure(
    std::array<double, 6> initial_deg = kinova::kFakeHomeDeg,
    const std::string & extra_hw_params = "")
  {
    sys.initial_deg = initial_deg;
    ASSERT_EQ(sys.on_init(parse(makeTestUrdf(extra_hw_params))), CallbackReturn::SUCCESS);
    ASSERT_EQ(sys.on_configure(lifecycle_state), CallbackReturn::SUCCESS);
    ASSERT_NE(sys.fake, nullptr);
    for (auto & state : sys.export_state_interfaces()) {
      state_names.push_back(state.get_name());
      states.emplace(state.get_name(), state);
    }
    for (auto & command : sys.export_command_interfaces()) {
      command_names.push_back(command.get_name());
      commands.emplace(command.get_name(), std::move(command));
    }
    ASSERT_EQ(sys.on_activate(lifecycle_state), CallbackReturn::SUCCESS);
  }

  return_type read(double period_s = 0.01)
  {
    return sys.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(period_s));
  }

  return_type write()
  {
    return sys.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  }

  /// prepare + perform, as the controller manager does; perform only if prepare succeeds.
  return_type switchInterfaces(
    const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    const return_type prepared = sys.prepare_command_mode_switch(start, stop);
    if (prepared != return_type::OK) {
      return prepared;
    }
    return sys.perform_command_mode_switch(start, stop);
  }

  void claim(const std::vector<std::string> & names)
  {
    ASSERT_EQ(switchInterfaces(names, {}), return_type::OK);
  }

  void release(const std::vector<std::string> & names)
  {
    ASSERT_EQ(switchInterfaces({}, names), return_type::OK);
  }

  void setCommand(const std::string & name, double value)
  {
    commands.at(name).set_value(value);
  }

  double state(const std::string & name) const
  {
    return states.at(name).get_value();
  }

  TestableKinovaSystem sys;
  rclcpp_lifecycle::State lifecycle_state;
  std::vector<std::string> state_names;
  std::vector<std::string> command_names;
  std::map<std::string, hardware_interface::StateInterface> states;
  std::map<std::string, hardware_interface::CommandInterface> commands;
};

}  // namespace

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
