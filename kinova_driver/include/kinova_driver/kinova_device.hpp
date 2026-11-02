#ifndef KINOVA_DRIVER__KINOVA_DEVICE_HPP_
#define KINOVA_DRIVER__KINOVA_DEVICE_HPP_

#include <kinova/KinovaTypes.h>

#include "kinova_driver/kinova_ros_types.h"

namespace kinova
{

/// The part of the Kinova arm API that the ros2_control hardware interface uses. Implemented by
/// KinovaComm (real arm) and FakeKinovaDevice (tests and use_fake_device).
///
/// Units are those of the Kinova API: degrees, deg/s, N*m and finger turns (0...6800).
///
/// Not to be confused with the vendor struct ::KinovaDevice (a USB device descriptor).
class KinovaDevice
{
public:
  virtual ~KinovaDevice() = default;

  /// Joint angles in degrees.
  virtual void getJointAngles(KinovaAngles & angles) = 0;
  /// Joint velocities in deg/s, after the firmware's factor of two has been corrected.
  virtual void getJointVelocities(KinovaAngles & velocities) = 0;
  /// Joint torques in N*m.
  virtual void getJointTorques(KinovaAngles & torques) = 0;
  /// Finger positions in turns.
  virtual void getFingerPositions(FingerAngles & fingers) = 0;

  virtual void setAngularControl() = 0;
  /// Stream one velocity point (deg/s) together with the finger target (turns).
  virtual void setJointVelocitiesWithFingerPositions(
    const AngularInfo & joint_vel_deg_s, const FingerAngles & fingers_turns) = 0;
  virtual void setFingerPositions(
    const FingerAngles & fingers, int timeout = 0, bool push = true) = 0;
  virtual void eraseAllTrajectories() = 0;
  virtual void homeArm() = 0;

  virtual void startAPI() = 0;
  virtual void stopAPI() = 0;
  virtual bool isStopped() = 0;
};

}  // namespace kinova

#endif  // KINOVA_DRIVER__KINOVA_DEVICE_HPP_
