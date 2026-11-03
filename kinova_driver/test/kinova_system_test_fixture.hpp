#ifndef KINOVA_DRIVER__TEST__KINOVA_SYSTEM_TEST_FIXTURE_HPP_
#define KINOVA_DRIVER__TEST__KINOVA_SYSTEM_TEST_FIXTURE_HPP_

// Shared by the KinovaSystem test executables. Each executable gets its own copy of these
// (anonymous namespace).

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

#endif  // KINOVA_DRIVER__TEST__KINOVA_SYSTEM_TEST_FIXTURE_HPP_
