#ifndef KINOVA_DRIVER__FAKE_KINOVA_DEVICE_HPP_
#define KINOVA_DRIVER__FAKE_KINOVA_DEVICE_HPP_

#include <array>
#include <functional>
#include <mutex>

#include "kinova_driver/conversions.hpp"
#include "kinova_driver/kinova_device.hpp"

namespace kinova
{

/// The "Home" pose of the j2n6s200 SRDF, in degrees.
inline const std::array<double, 6> kFakeHomeDeg = {
  kinova_driver::radToDeg(4.8046852), kinova_driver::radToDeg(2.92482),
  kinova_driver::radToDeg(1.002), kinova_driver::radToDeg(4.2031852),
  kinova_driver::radToDeg(1.4458), kinova_driver::radToDeg(1.3233)};

/// A 6-joint arm that integrates the commanded joint velocity. Like the real arm, a velocity
/// command only stays in effect for a short time (kCommandTimeoutS) after it was received, so
/// the command has to be streamed.
///
/// All methods are thread safe. Time comes from now_s so that tests can use a manual clock.
class FakeKinovaDevice : public KinovaDevice
{
public:
  /// A velocity command stays in effect for this long after it was received.
  static constexpr double kCommandTimeoutS = 0.05;

  /// Seconds since an arbitrary epoch, from std::chrono::steady_clock.
  static double steadyClockSeconds();

  explicit FakeKinovaDevice(
    std::array<double, 6> initial_angles_deg = kFakeHomeDeg,
    std::function<double()> now_s = &FakeKinovaDevice::steadyClockSeconds);

  void getJointAngles(KinovaAngles & angles) override;
  void getJointVelocities(KinovaAngles & velocities) override;
  void getJointTorques(KinovaAngles & torques) override;
  void getFingerPositions(FingerAngles & fingers) override;

  void setAngularControl() override;
  void setJointVelocitiesWithFingerPositions(
    const AngularInfo & joint_vel_deg_s, const FingerAngles & fingers_turns) override;
  void setFingerPositions(
    const FingerAngles & fingers, int timeout = 0, bool push = true) override;
  void eraseAllTrajectories() override;
  void homeArm() override;

  void startAPI() override;
  void stopAPI() override;
  bool isStopped() override;

  // Test accessors. A finger command is a call to either setFingerPositions or
  // setJointVelocitiesWithFingerPositions that was not ignored.
  int velocityCommandCount() const;
  AngularInfo lastVelocityCommand() const;
  int fingerCommandCount() const;
  FingerAngles lastFingerCommand() const;
  int homeCount() const;
  /// Teleport the arm. Does not count as a command.
  void setJointAnglesDeg(const std::array<double, 6> & angles_deg);
  /// Set the finger positions directly. Does not count as a command.
  void setFingerTurns(double f1, double f2);

private:
  /// Integrate the commanded velocity up to now, but not beyond the command timeout.
  /// Must be called with mutex_ held, at the start of every public method.
  void advance(double now);

  mutable std::mutex mutex_;
  std::function<double()> now_s_;
  std::array<double, 6> initial_angles_deg_;
  std::array<double, 6> angles_deg_;
  /// The velocity command in effect while it has not timed out; zero before the first command
  /// and after stopAPI().
  AngularInfo velocity_deg_s_ = {};
  double last_command_s_;
  double last_advance_s_;
  FingerAngles fingers_turns_;
  bool stopped_ = false;

  int velocity_command_count_ = 0;
  AngularInfo last_velocity_command_ = {};
  int finger_command_count_ = 0;
  FingerAngles last_finger_command_;
  int home_count_ = 0;
};

}  // namespace kinova

#endif  // KINOVA_DRIVER__FAKE_KINOVA_DEVICE_HPP_
