#ifndef KINOVA_DRIVER__KINOVA_SYSTEM_HPP_
#define KINOVA_DRIVER__KINOVA_SYSTEM_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "kinova_driver/kinova_comm_config.hpp"
#include "kinova_driver/kinova_device.hpp"
#include "kinova_msgs/srv/home_arm.hpp"
#include "kinova_msgs/srv/start.hpp"
#include "kinova_msgs/srv/stop.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace kinova_driver
{

/// ros2_control system for a Kinova arm.
///
/// The <ros2_control> block lists, in order: the arm joints (`..joint_1` to `..joint_<dof>`,
/// command {velocity}, state {position, velocity, effort}), the fingers (`..joint_finger_<k>`,
/// state {position, velocity}; finger 1 also has command {position}), then the finger tips
/// (`..joint_finger_tip_<k>`, state {position}). dof and the finger count come from the
/// robot_type hardware parameter.
///
/// write() sends nothing unless the component is active (between on_activate and
/// on_deactivate). Arm velocity is streamed to the arm at every write() while all arm velocity
/// interfaces are claimed, and one zero-velocity point is sent when they are released. The
/// finger 1 position command is applied to every finger. on_shutdown and the destructor send
/// one zero point and erase the trajectories if the component is still active.
///
/// With the enable_services hardware parameter (default true), on_configure also starts a node
/// `<robot_type>_driver` with the services `~/in/home_arm`, `~/in/start` and `~/in/stop`, spun
/// on its own thread until on_cleanup, on_shutdown or destruction. All device access is guarded
/// by one mutex, because the service callbacks run on that thread.
class KinovaSystem : public hardware_interface::SystemInterface
{
public:
  ~KinovaSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  /// Silences write(), zeroes the arm commands and sends one zero point (and erases the
  /// trajectories) if the device is still there. Always succeeds, so the component ends up UNCONFIGURED.
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// Rejects a switch that would leave only some of the arm velocity interfaces claimed.
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  /// Applies stop_interfaces, then start_interfaces.
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Service handlers; public so tests can call them directly. Each answers
  /// "Rejected: hardware not configured" when there is no device.
  /// Refuses while the arm velocity interfaces are claimed by a controller.
  void handleHomeArm(
    const std::shared_ptr<kinova_msgs::srv::HomeArm::Request> request,
    std::shared_ptr<kinova_msgs::srv::HomeArm::Response> response);
  /// Stops the arm API; write() sends nothing until handleStart.
  void handleStop(
    const std::shared_ptr<kinova_msgs::srv::Stop::Request> request,
    std::shared_ptr<kinova_msgs::srv::Stop::Response> response);
  void handleStart(
    const std::shared_ptr<kinova_msgs::srv::Start::Request> request,
    std::shared_ptr<kinova_msgs::srv::Start::Response> response);

protected:
  /// Called by on_configure. Returns a FakeKinovaDevice if use_fake_device is set, otherwise
  /// connects to the arm through KinovaComm. May throw std::exception.
  virtual std::unique_ptr<kinova::KinovaDevice> createDevice();

  /// True while every arm velocity interface is claimed. Read from other threads.
  std::atomic<bool> arm_velocity_claimed_{false};

private:
  /// read() without taking device_mutex_; the caller holds it.
  hardware_interface::return_type readLocked(const rclcpp::Duration & period);
  /// Create the service node and its executor thread, or stop and join them. Neither is called
  /// with device_mutex_ held. stopServices is safe to call twice.
  bool startServices();
  void stopServices();

  /// If the component is active: clear active_ and, unless the API is stopped, send one
  /// zero-velocity point and erase all trajectories. Never throws. Caller holds device_mutex_.
  void stopArmIfActiveLocked() noexcept;

  /// Finger targets in turns: target for fingers 1 and 2, and for finger 3 if the hand has one.
  kinova::FingerAngles fingerTargets(double turns) const;

  // Hardware parameters.
  kinova::KinovaCommConfig comm_config_;
  bool use_fake_device_ = false;
  double finger_max_turn_ = 6800.0;
  /// Start the in/home_arm, in/start and in/stop services in on_configure.
  bool enable_services_ = true;

  std::size_t dof_ = 0;
  std::size_t fingers_ = 0;
  /// Guards device_ and its lifetime against the service thread. Never held while joining it.
  std::mutex device_mutex_;
  std::unique_ptr<kinova::KinovaDevice> device_;

  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spinning_{false};
  rclcpp::Service<kinova_msgs::srv::HomeArm>::SharedPtr home_arm_service_;
  rclcpp::Service<kinova_msgs::srv::Start>::SharedPtr start_service_;
  rclcpp::Service<kinova_msgs::srv::Stop>::SharedPtr stop_service_;

  // State interface storage, in rad, rad/s and N*m.
  std::vector<double> arm_position_;
  std::vector<double> arm_velocity_;
  std::vector<double> arm_effort_;
  std::vector<double> finger_position_;
  std::vector<double> finger_velocity_;
  std::vector<double> tip_position_;
  /// False until the first read() after on_configure; that read does not unwrap.
  bool have_position_ = false;

  // Command interface storage, in rad/s and rad.
  std::vector<double> arm_velocity_command_;
  double finger_position_command_ = 0.0;
  /// Velocity command limits from the URDF, in rad/s. +-infinity when not given.
  std::vector<double> velocity_min_;
  std::vector<double> velocity_max_;

  // Command interface claims.
  std::vector<std::string> arm_command_names_;
  std::string finger_command_name_;
  std::set<std::string> claimed_;
  bool finger_claimed_ = false;
  /// Set when the arm velocity interfaces are released; write() then sends one zero point.
  bool send_zero_once_ = false;
  /// True from the end of a successful on_activate to the start of on_configure,
  /// on_deactivate, on_cleanup, on_shutdown or on_error. write() sends nothing while it is false.
  std::atomic<bool> active_{false};

  /// Finger target in turns: the measured position at on_activate, then the clamped command.
  double finger_target_turns_ = 0.0;
  double last_sent_finger_turns_ = 0.0;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace kinova_driver

#endif  // KINOVA_DRIVER__KINOVA_SYSTEM_HPP_
