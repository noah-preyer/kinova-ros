#include "kinova_driver/fake_kinova_device.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace kinova
{

namespace
{

void zeroFingers(FingerAngles & fingers)
{
  fingers.Finger1 = 0.0f;
  fingers.Finger2 = 0.0f;
  fingers.Finger3 = 0.0f;
}

void copyAngles(const std::array<double, 6> & from, KinovaAngles & to)
{
  to.Actuator1 = static_cast<float>(from[0]);
  to.Actuator2 = static_cast<float>(from[1]);
  to.Actuator3 = static_cast<float>(from[2]);
  to.Actuator4 = static_cast<float>(from[3]);
  to.Actuator5 = static_cast<float>(from[4]);
  to.Actuator6 = static_cast<float>(from[5]);
  to.Actuator7 = 0.0f;
}

}  // namespace

double FakeKinovaDevice::steadyClockSeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

FakeKinovaDevice::FakeKinovaDevice(
  std::array<double, 6> initial_angles_deg, std::function<double()> now_s)
: now_s_(std::move(now_s)),
  initial_angles_deg_(initial_angles_deg),
  angles_deg_(initial_angles_deg),
  last_command_s_(now_s_()),
  last_advance_s_(last_command_s_)
{
  zeroFingers(fingers_turns_);
  zeroFingers(last_finger_command_);
}

void FakeKinovaDevice::advance(double now)
{
  // The part of [last_advance, now] that lies inside the command timeout.
  const double end = std::min(now, last_command_s_ + kCommandTimeoutS);
  if (end > last_advance_s_) {
    const double dt = end - last_advance_s_;
    angles_deg_[0] += velocity_deg_s_.Actuator1 * dt;
    angles_deg_[1] += velocity_deg_s_.Actuator2 * dt;
    angles_deg_[2] += velocity_deg_s_.Actuator3 * dt;
    angles_deg_[3] += velocity_deg_s_.Actuator4 * dt;
    angles_deg_[4] += velocity_deg_s_.Actuator5 * dt;
    angles_deg_[5] += velocity_deg_s_.Actuator6 * dt;
  }
  last_advance_s_ = now;
}

void FakeKinovaDevice::getJointAngles(KinovaAngles & angles)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  copyAngles(angles_deg_, angles);
}

void FakeKinovaDevice::getJointVelocities(KinovaAngles & velocities)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = now_s_();
  advance(now);
  if (now <= last_command_s_ + kCommandTimeoutS) {
    velocities.Actuator1 = velocity_deg_s_.Actuator1;
    velocities.Actuator2 = velocity_deg_s_.Actuator2;
    velocities.Actuator3 = velocity_deg_s_.Actuator3;
    velocities.Actuator4 = velocity_deg_s_.Actuator4;
    velocities.Actuator5 = velocity_deg_s_.Actuator5;
    velocities.Actuator6 = velocity_deg_s_.Actuator6;
    velocities.Actuator7 = 0.0f;
  } else {
    copyAngles({0, 0, 0, 0, 0, 0}, velocities);
  }
}

void FakeKinovaDevice::getJointTorques(KinovaAngles & torques)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  copyAngles({0, 0, 0, 0, 0, 0}, torques);
}

void FakeKinovaDevice::getFingerPositions(FingerAngles & fingers)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  fingers = fingers_turns_;
}

void FakeKinovaDevice::setAngularControl()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
}

void FakeKinovaDevice::setJointVelocitiesWithFingerPositions(
  const AngularInfo & joint_vel_deg_s, const FingerAngles & fingers_turns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double now = now_s_();
  advance(now);
  if (stopped_) {
    return;
  }
  velocity_deg_s_ = joint_vel_deg_s;
  last_command_s_ = now;
  last_velocity_command_ = joint_vel_deg_s;
  ++velocity_command_count_;

  fingers_turns_ = fingers_turns;
  last_finger_command_ = fingers_turns;
  ++finger_command_count_;
}

void FakeKinovaDevice::setFingerPositions(const FingerAngles & fingers, int /*timeout*/, bool /*push*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  if (stopped_) {
    return;
  }
  fingers_turns_ = fingers;
  last_finger_command_ = fingers;
  ++finger_command_count_;
}

void FakeKinovaDevice::eraseAllTrajectories()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
}

void FakeKinovaDevice::homeArm()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  angles_deg_ = initial_angles_deg_;
  ++home_count_;
}

void FakeKinovaDevice::startAPI()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  stopped_ = false;
}

void FakeKinovaDevice::stopAPI()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  stopped_ = true;
  velocity_deg_s_ = {};
}

bool FakeKinovaDevice::isStopped()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  return stopped_;
}

int FakeKinovaDevice::velocityCommandCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return velocity_command_count_;
}

AngularInfo FakeKinovaDevice::lastVelocityCommand() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_velocity_command_;
}

int FakeKinovaDevice::fingerCommandCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return finger_command_count_;
}

FingerAngles FakeKinovaDevice::lastFingerCommand() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_finger_command_;
}

int FakeKinovaDevice::homeCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return home_count_;
}

void FakeKinovaDevice::setJointAnglesDeg(const std::array<double, 6> & angles_deg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  angles_deg_ = angles_deg;
}

void FakeKinovaDevice::setFingerTurns(double f1, double f2)
{
  std::lock_guard<std::mutex> lock(mutex_);
  advance(now_s_());
  fingers_turns_.Finger1 = static_cast<float>(f1);
  fingers_turns_.Finger2 = static_cast<float>(f2);
}

}  // namespace kinova
