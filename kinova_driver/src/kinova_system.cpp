#include "kinova_driver/kinova_system.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <sstream>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinova_driver/conversions.hpp"
#include "kinova_driver/fake_kinova_device.hpp"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_ros_types.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"

namespace kinova_driver
{

namespace
{

using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::return_type;

constexpr std::size_t kMaxJoints = 7;
constexpr std::size_t kMaxFingers = 3;
constexpr float AngularInfo::* kActuators[kMaxJoints] = {
  &AngularInfo::Actuator1, &AngularInfo::Actuator2, &AngularInfo::Actuator3,
  &AngularInfo::Actuator4, &AngularInfo::Actuator5, &AngularInfo::Actuator6,
  &AngularInfo::Actuator7};
constexpr float FingersPosition::* kFingers[kMaxFingers] = {
  &FingersPosition::Finger1, &FingersPosition::Finger2, &FingersPosition::Finger3};
/// setFingerPositions is only sent alone when the target moved by more than this many turns.
constexpr double kFingerDeadbandTurns = 1.0;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("KinovaSystem");
}

bool endsWith(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string join(const std::vector<std::string> & names)
{
  std::string joined;
  for (const auto & name : names) {
    joined += (joined.empty() ? "" : ", ") + name;
  }
  return joined;
}

std::vector<std::string> interfaceNames(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  std::vector<std::string> names;
  for (const auto & interface : interfaces) {
    names.push_back(interface.name);
  }
  return names;
}

/// True if interfaces holds exactly the names in expected, in any order.
bool hasInterfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  std::vector<std::string> expected)
{
  std::vector<std::string> names = interfaceNames(interfaces);
  std::sort(names.begin(), names.end());
  std::sort(expected.begin(), expected.end());
  return names == expected;
}

bool parseDouble(const std::string & text, double & value)
{
  try {
    value = hardware_interface::stod(text);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

/// A velocity limit; empty text means no limit (unlimited).
bool parseLimit(const std::string & text, double unlimited, double & value)
{
  if (text.empty()) {
    value = unlimited;
    return true;
  }
  return parseDouble(text, value) && !std::isnan(value);
}

/// Accepts true/false as written by hand or by xacro (True/False).
bool parseBool(const std::string & text, bool & value)
{
  value = text == "true" || text == "True";
  return value || text == "false" || text == "False";
}

bool parseInt(const std::string & text, int & value)
{
  std::istringstream stream(text);
  stream >> value;
  return !stream.fail() && stream.eof();
}

}  // namespace

CallbackReturn KinovaSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Hardware parameters. Absent ones keep their defaults.
  const auto & params = info_.hardware_parameters;
  bool params_ok = true;
  auto param = [&](const std::string & name, auto & value, auto parse) {
      const auto it = params.find(name);
      if (it != params.end() && !parse(it->second, value)) {
        RCLCPP_ERROR(
          logger(), "Invalid value '%s' for hardware parameter %s", it->second.c_str(),
          name.c_str());
        params_ok = false;
      }
    };
  auto parse_string = [](const std::string & text, std::string & value) {
      value = text;
      return true;
    };
  comm_config_ = kinova::KinovaCommConfig();
  param("robot_type", comm_config_.robot_type, parse_string);
  param("use_fake_device", use_fake_device_, parseBool);
  param("serial_number", comm_config_.serial_number, parse_string);
  param("connection_type", comm_config_.connection_type, parse_string);
  param("local_machine_ip", comm_config_.local_machine_ip, parse_string);
  param("subnet_mask", comm_config_.subnet_mask, parse_string);
  param("local_cmd_port", comm_config_.local_cmd_port, parseInt);
  param("local_broadcast_port", comm_config_.local_broadcast_port, parseInt);
  param("finger_max_turn", finger_max_turn_, parseDouble);
  param("init_fingers_on_configure", comm_config_.movement_on_start, parseBool);
  param("enable_services", enable_services_, parseBool);
  if (!params_ok) {
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(finger_max_turn_) || finger_max_turn_ < 0.0) {
    RCLCPP_ERROR(logger(), "Hardware parameter finger_max_turn must be >= 0");
    return CallbackReturn::ERROR;
  }

  const std::string & robot_type = comm_config_.robot_type;
  if (!kinova::valid_kinovaRobotType(robot_type)) {
    RCLCPP_ERROR(
      logger(), "Hardware parameter robot_type '%s' is missing or invalid", robot_type.c_str());
    return CallbackReturn::ERROR;
  }
  const int dof = robot_type[3] - '0';
  const int fingers = robot_type[5] - '0';
  if (dof < 1 || dof > static_cast<int>(kMaxJoints) ||
    fingers < 1 || fingers > static_cast<int>(kMaxFingers))
  {
    RCLCPP_ERROR(
      logger(), "robot_type '%s' must give 1-%zu arm joints and 1-%zu fingers",
      robot_type.c_str(), kMaxJoints, kMaxFingers);
    return CallbackReturn::ERROR;
  }
  dof_ = static_cast<std::size_t>(dof);
  fingers_ = static_cast<std::size_t>(fingers);

  // Joints: dof arm joints, then fingers fingers, then fingers finger tips.
  const auto & joints = info_.joints;
  if (joints.size() != dof_ + 2 * fingers_) {
    RCLCPP_ERROR(
      logger(), "robot_type '%s' needs %zu joints in <ros2_control> (%zu arm, %zu fingers, "
      "%zu finger tips), got %zu", robot_type.c_str(), dof_ + 2 * fingers_, dof_, fingers_,
      fingers_, joints.size());
    return CallbackReturn::ERROR;
  }
  auto check_joint = [&](
    std::size_t index, const std::string & suffix,
    const std::vector<std::string> & commands, const std::vector<std::string> & states) {
      const auto & joint = joints[index];
      if (endsWith(joint.name, suffix) && hasInterfaces(joint.command_interfaces, commands) &&
        hasInterfaces(joint.state_interfaces, states))
      {
        return true;
      }
      RCLCPP_ERROR(
        logger(), "Joint %zu '%s' in <ros2_control> must end with '%s' and have command "
        "interfaces {%s} and state interfaces {%s}; it has {%s} and {%s}", index + 1,
        joint.name.c_str(), suffix.c_str(), join(commands).c_str(), join(states).c_str(),
        join(interfaceNames(joint.command_interfaces)).c_str(),
        join(interfaceNames(joint.state_interfaces)).c_str());
      return false;
    };

  velocity_min_.assign(dof_, 0.0);
  velocity_max_.assign(dof_, 0.0);
  arm_command_names_.clear();
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!check_joint(
        i, "joint_" + std::to_string(i + 1), {HW_IF_VELOCITY},
        {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT}))
    {
      return CallbackReturn::ERROR;
    }
    const auto & command = joints[i].command_interfaces[0];
    if (!parseLimit(command.min, -std::numeric_limits<double>::infinity(), velocity_min_[i]) ||
      !parseLimit(command.max, std::numeric_limits<double>::infinity(), velocity_max_[i]) ||
      velocity_min_[i] > velocity_max_[i])
    {
      RCLCPP_ERROR(
        logger(), "Joint '%s' has invalid velocity limits min '%s', max '%s'",
        joints[i].name.c_str(), command.min.c_str(), command.max.c_str());
      return CallbackReturn::ERROR;
    }
    arm_command_names_.push_back(joints[i].name + "/" + HW_IF_VELOCITY);
  }
  for (std::size_t k = 0; k < fingers_; ++k) {
    const std::vector<std::string> commands =
      k == 0 ? std::vector<std::string>{HW_IF_POSITION} : std::vector<std::string>{};
    if (!check_joint(
        dof_ + k, "joint_finger_" + std::to_string(k + 1), commands,
        {HW_IF_POSITION, HW_IF_VELOCITY}))
    {
      return CallbackReturn::ERROR;
    }
  }
  for (std::size_t k = 0; k < fingers_; ++k) {
    if (!check_joint(
        dof_ + fingers_ + k, "joint_finger_tip_" + std::to_string(k + 1), {}, {HW_IF_POSITION}))
    {
      return CallbackReturn::ERROR;
    }
  }
  finger_command_name_ = joints[dof_].name + "/" + HW_IF_POSITION;

  // The exported interfaces point into these vectors, so they are not resized after this.
  arm_position_.assign(dof_, 0.0);
  arm_velocity_.assign(dof_, 0.0);
  arm_effort_.assign(dof_, 0.0);
  finger_position_.assign(fingers_, 0.0);
  finger_velocity_.assign(fingers_, 0.0);
  tip_position_.assign(fingers_, 0.0);
  arm_velocity_command_.assign(dof_, 0.0);
  finger_position_command_ = 0.0;
  return CallbackReturn::SUCCESS;
}

std::unique_ptr<kinova::KinovaDevice> KinovaSystem::createDevice()
{
  if (use_fake_device_) {
    RCLCPP_INFO(logger(), "Using the fake Kinova device (use_fake_device is true)");
    return std::make_unique<kinova::FakeKinovaDevice>();
  }
  kinova::KinovaCommConfig config = comm_config_;
  config.api_library_dir = ament_index_cpp::get_package_prefix("kinova_driver") + "/lib";
  return std::make_unique<kinova::KinovaComm>(config);
}

KinovaSystem::~KinovaSystem()
{
  stopServices();
  std::lock_guard<std::mutex> lock(device_mutex_);
  stopArmIfActiveLocked();
  device_.reset();
}

void KinovaSystem::stopArmIfActiveLocked() noexcept
{
  // Silence write() first. Humble can tear the hardware down (e.g. Ctrl-C) without calling
  // on_deactivate, so an active arm gets its zero point and trajectory erase here.
  const bool was_active = active_.exchange(false);
  if (!was_active || !device_) {
    return;
  }
  try {
    if (device_->isStopped()) {
      return;
    }
    device_->setJointVelocitiesWithFingerPositions(
      AngularInfo{}, fingerTargets(finger_target_turns_));
    device_->eraseAllTrajectories();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not stop the Kinova arm on shutdown: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger(), "Could not stop the Kinova arm on shutdown");
  }
}

bool KinovaSystem::startServices()
{
  try {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    service_node_ = std::make_shared<rclcpp::Node>(
      comm_config_.robot_type + "_driver", options);
    home_arm_service_ = service_node_->create_service<kinova_msgs::srv::HomeArm>(
      "~/in/home_arm", [this](
        const std::shared_ptr<kinova_msgs::srv::HomeArm::Request> request,
        std::shared_ptr<kinova_msgs::srv::HomeArm::Response> response) {
        handleHomeArm(request, response);
      });
    start_service_ = service_node_->create_service<kinova_msgs::srv::Start>(
      "~/in/start", [this](
        const std::shared_ptr<kinova_msgs::srv::Start::Request> request,
        std::shared_ptr<kinova_msgs::srv::Start::Response> response) {
        handleStart(request, response);
      });
    stop_service_ = service_node_->create_service<kinova_msgs::srv::Stop>(
      "~/in/stop", [this](
        const std::shared_ptr<kinova_msgs::srv::Stop::Request> request,
        std::shared_ptr<kinova_msgs::srv::Stop::Response> response) {
        handleStop(request, response);
      });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(service_node_);
    stop_spinning_ = false;
    // spin_once with a timeout instead of spin(): a cancel() that arrives before spin() starts
    // would otherwise be lost and the join would hang.
    spin_thread_ = std::thread(
      [this, executor = executor_] {
        while (!stop_spinning_ && rclcpp::ok()) {
          executor->spin_once(std::chrono::milliseconds(50));
        }
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not start the driver services: %s", e.what());
    stopServices();
    return false;
  }
  return true;
}

void KinovaSystem::stopServices()
{
  stop_spinning_ = true;
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  home_arm_service_.reset();
  start_service_.reset();
  stop_service_.reset();
  executor_.reset();
  service_node_.reset();
}

void KinovaSystem::handleHomeArm(
  const std::shared_ptr<kinova_msgs::srv::HomeArm::Request> /*request*/,
  std::shared_ptr<kinova_msgs::srv::HomeArm::Response> response)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!device_) {
    response->homearm_result = "Rejected: hardware not configured";
    return;
  }
  if (arm_velocity_claimed_) {
    response->homearm_result = "Rejected: deactivate arm_controller before homing";
    return;
  }
  try {
    if (device_->isStopped()) {
      response->homearm_result = "Rejected: arm stopped (call in/start first)";
      return;
    }
    device_->homeArm();
    response->homearm_result = "KINOVA ARM HAS BEEN RETURNED HOME";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not home the Kinova arm: %s", e.what());
    response->homearm_result = std::string("Failed: ") + e.what();
  }
}

void KinovaSystem::handleStop(
  const std::shared_ptr<kinova_msgs::srv::Stop::Request> /*request*/,
  std::shared_ptr<kinova_msgs::srv::Stop::Response> response)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!device_) {
    response->stop_result = "Rejected: hardware not configured";
    return;
  }
  try {
    device_->stopAPI();
    response->stop_result = "Arm stopped";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not stop the Kinova arm: %s", e.what());
    response->stop_result = std::string("Failed: ") + e.what();
  }
}

void KinovaSystem::handleStart(
  const std::shared_ptr<kinova_msgs::srv::Start::Request> /*request*/,
  std::shared_ptr<kinova_msgs::srv::Start::Response> response)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!device_) {
    response->start_result = "Rejected: hardware not configured";
    return;
  }
  // Starting with arm_controller active would resume streaming against its pre-stop reference.
  if (arm_velocity_claimed_) {
    response->start_result = "Rejected: deactivate arm_controller before starting";
    return;
  }
  try {
    device_->startAPI();
    response->start_result = "Arm started";
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not start the Kinova arm: %s", e.what());
    response->start_result = std::string("Failed: ") + e.what();
  }
}

CallbackReturn KinovaSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // After an error the component is reconfigured without on_deactivate; stay silent until
  // on_activate.
  active_ = false;
  stopServices();
  // Only one connection to the arm at a time: the old device closes the API when destroyed.
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    device_.reset();
  }
  std::unique_ptr<kinova::KinovaDevice> device;
  try {
    device = createDevice();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger(), "Could not connect to the Kinova arm: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (!device) {
    RCLCPP_ERROR(logger(), "Could not connect to the Kinova arm: no device");
    return CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    device_ = std::move(device);
    // Make the state interfaces hold real values before anything reads them.
    have_position_ = false;
    if (readLocked(rclcpp::Duration(0, 0)) != return_type::OK) {
      device_.reset();
      return CallbackReturn::ERROR;
    }
  }
  if (enable_services_ && !startServices()) {
    std::lock_guard<std::mutex> lock(device_mutex_);
    device_.reset();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinovaSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // A claim that survived on_error/on_configure would start streaming the stale reference.
  if (arm_velocity_claimed_) {
    RCLCPP_ERROR(
      logger(), "Could not activate the Kinova arm: arm interfaces still claimed by a "
      "controller; deactivate it before activating hardware");
    return CallbackReturn::ERROR;
  }
  kinova::FingerAngles fingers;
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    try {
      device_->setAngularControl();
      device_->getFingerPositions(fingers);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Could not activate the Kinova arm: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  std::fill(arm_velocity_command_.begin(), arm_velocity_command_.end(), 0.0);
  finger_target_turns_ = fingers.Finger1;
  last_sent_finger_turns_ = finger_target_turns_;
  finger_position_command_ = fingerTurnsToRad(finger_target_turns_);
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinovaSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Silence write() first. The zero point below goes to the device directly.
  active_ = false;
  {
    std::lock_guard<std::mutex> lock(device_mutex_);
    try {
      device_->setJointVelocitiesWithFingerPositions(
        AngularInfo{}, fingerTargets(finger_target_turns_));
      device_->eraseAllTrajectories();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Could not stop the Kinova arm: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  last_sent_finger_turns_ = finger_target_turns_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinovaSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  stopServices();
  std::lock_guard<std::mutex> lock(device_mutex_);
  device_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinovaSystem::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopServices();
  std::lock_guard<std::mutex> lock(device_mutex_);
  stopArmIfActiveLocked();
  device_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinovaSystem::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  std::fill(arm_velocity_command_.begin(), arm_velocity_command_.end(), 0.0);
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (device_) {
    try {
      device_->setJointVelocitiesWithFingerPositions(
        AngularInfo{}, fingerTargets(finger_target_turns_));
      device_->eraseAllTrajectories();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Could not stop the Kinova arm after an error: %s", e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KinovaSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < dof_; ++i) {
    const std::string & joint = info_.joints[i].name;
    interfaces.emplace_back(joint, HW_IF_POSITION, &arm_position_[i]);
    interfaces.emplace_back(joint, HW_IF_VELOCITY, &arm_velocity_[i]);
    interfaces.emplace_back(joint, HW_IF_EFFORT, &arm_effort_[i]);
  }
  for (std::size_t k = 0; k < fingers_; ++k) {
    const std::string & joint = info_.joints[dof_ + k].name;
    interfaces.emplace_back(joint, HW_IF_POSITION, &finger_position_[k]);
    interfaces.emplace_back(joint, HW_IF_VELOCITY, &finger_velocity_[k]);
  }
  for (std::size_t k = 0; k < fingers_; ++k) {
    const std::string & joint = info_.joints[dof_ + fingers_ + k].name;
    interfaces.emplace_back(joint, HW_IF_POSITION, &tip_position_[k]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> KinovaSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < dof_; ++i) {
    interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &arm_velocity_command_[i]);
  }
  interfaces.emplace_back(info_.joints[dof_].name, HW_IF_POSITION, &finger_position_command_);
  return interfaces;
}

return_type KinovaSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::set<std::string> claimed = claimed_;
  for (const auto & name : stop_interfaces) {
    claimed.erase(name);
  }
  for (const auto & name : start_interfaces) {
    claimed.insert(name);
  }
  const auto arm_claimed = std::count_if(
    arm_command_names_.begin(), arm_command_names_.end(),
    [&claimed](const std::string & name) {return claimed.count(name) > 0;});
  if (arm_claimed != 0 && static_cast<std::size_t>(arm_claimed) != dof_) {
    RCLCPP_ERROR(
      logger(), "Rejected command mode switch: it would claim %zu of the %zu arm velocity "
      "interfaces, but a controller must claim all of them", static_cast<std::size_t>(arm_claimed),
      dof_);
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type KinovaSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  auto is_arm = [this](const std::string & name) {
      return std::find(arm_command_names_.begin(), arm_command_names_.end(), name) !=
             arm_command_names_.end();
    };
  auto is_ours = [&](const std::string & name) {
      return is_arm(name) || name == finger_command_name_;
    };

  for (const auto & name : stop_interfaces) {
    if (claimed_.erase(name) > 0 && is_arm(name)) {
      std::fill(arm_velocity_command_.begin(), arm_velocity_command_.end(), 0.0);
      send_zero_once_ = true;
    }
  }
  for (const auto & name : start_interfaces) {
    if (is_ours(name)) {
      claimed_.insert(name);
    }
  }

  arm_velocity_claimed_ = std::all_of(
    arm_command_names_.begin(), arm_command_names_.end(),
    [this](const std::string & name) {return claimed_.count(name) > 0;});
  finger_claimed_ = claimed_.count(finger_command_name_) > 0;
  return return_type::OK;
}

return_type KinovaSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(device_mutex_);
  return readLocked(period);
}

return_type KinovaSystem::readLocked(const rclcpp::Duration & period)
{
  kinova::KinovaAngles angles;
  kinova::KinovaAngles velocities;
  kinova::KinovaAngles torques;
  kinova::FingerAngles fingers;
  try {
    device_->getJointAngles(angles);
    device_->getJointVelocities(velocities);
    device_->getJointTorques(torques);
    device_->getFingerPositions(fingers);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger(), steady_clock_, 1000, "Could not read the Kinova arm state: %s", e.what());
    return return_type::ERROR;
  }

  for (std::size_t i = 0; i < dof_; ++i) {
    const double position = degToRad(angles.*kActuators[i]);
    arm_position_[i] = have_position_ ? unwrapNear(position, arm_position_[i]) : position;
    arm_velocity_[i] = kinovaVelocityToRad(velocities.*kActuators[i]);
    arm_effort_[i] = torques.*kActuators[i];
  }
  have_position_ = true;

  const double dt = period.seconds();
  for (std::size_t k = 0; k < fingers_; ++k) {
    const double position = fingerTurnsToRad(fingers.*kFingers[k]);
    finger_velocity_[k] = dt > 0.0 ? (position - finger_position_[k]) / dt : 0.0;
    finger_position_[k] = position;
    tip_position_[k] = 0.0;
  }
  return return_type::OK;
}

return_type KinovaSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ros2_control (Humble) also calls write() on an inactive component.
  if (!active_) {
    return return_type::OK;
  }
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!device_) {
    return return_type::OK;
  }
  try {
    if (device_->isStopped()) {
      return return_type::OK;
    }

    AngularInfo velocity_deg_s{};
    for (std::size_t i = 0; i < dof_; ++i) {
      double command = arm_velocity_command_[i];
      if (!std::isfinite(command)) {
        command = 0.0;
      }
      command = std::clamp(command, velocity_min_[i], velocity_max_[i]);
      velocity_deg_s.*kActuators[i] = static_cast<float>(radToDeg(command));
    }
    if (finger_claimed_ && std::isfinite(finger_position_command_)) {
      finger_target_turns_ =
        std::clamp(fingerRadToTurns(finger_position_command_), 0.0, finger_max_turn_);
    }
    const kinova::FingerAngles fingers = fingerTargets(finger_target_turns_);

    if (arm_velocity_claimed_) {
      device_->setJointVelocitiesWithFingerPositions(velocity_deg_s, fingers);
      last_sent_finger_turns_ = finger_target_turns_;
    } else if (send_zero_once_) {
      device_->setJointVelocitiesWithFingerPositions(AngularInfo{}, fingers);
      send_zero_once_ = false;
      last_sent_finger_turns_ = finger_target_turns_;
    } else if (finger_claimed_ &&
      std::abs(finger_target_turns_ - last_sent_finger_turns_) > kFingerDeadbandTurns)
    {
      device_->setFingerPositions(fingers);
      last_sent_finger_turns_ = finger_target_turns_;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger(), steady_clock_, 1000, "Could not command the Kinova arm: %s", e.what());
    return return_type::ERROR;
  }
  return return_type::OK;
}

kinova::FingerAngles KinovaSystem::fingerTargets(double turns) const
{
  const auto target = static_cast<float>(turns);
  kinova::FingerAngles fingers;
  fingers.Finger1 = target;
  fingers.Finger2 = target;
  fingers.Finger3 = fingers_ == 3 ? target : 0.0f;
  return fingers;
}

}  // namespace kinova_driver

PLUGINLIB_EXPORT_CLASS(kinova_driver::KinovaSystem, hardware_interface::SystemInterface)
