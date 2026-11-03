#ifndef KINOVA_DRIVER__TEST__KINOVA_TEST_URDF_HPP_
#define KINOVA_DRIVER__TEST__KINOVA_TEST_URDF_HPP_

#include <string>

/// A minimal URDF holding only the j2n6s200 <ros2_control> block of the interface contract,
/// with use_fake_device=true and enable_services=false. extra_hw_params is inserted verbatim
/// after the default hardware <param>s. drop_last_tip leaves out j2n6s200_joint_finger_tip_2.
inline std::string makeTestUrdf(
  const std::string & extra_hw_params = "", bool drop_last_tip = false)
{
  const std::string prefix = "j2n6s200_";

  auto arm_joint = [&](int n, const std::string & limit) {
      return
        "<joint name=\"" + prefix + "joint_" + std::to_string(n) + "\">"
        "<command_interface name=\"velocity\">"
        "<param name=\"min\">-" + limit + "</param>"
        "<param name=\"max\">" + limit + "</param>"
        "</command_interface>"
        "<state_interface name=\"position\"/>"
        "<state_interface name=\"velocity\"/>"
        "<state_interface name=\"effort\"/>"
        "</joint>";
    };

  std::string urdf =
    "<robot name=\"t\">"
    "<ros2_control name=\"j2n6s200_system\" type=\"system\">"
    "<hardware>"
    "<plugin>kinova_driver/KinovaSystem</plugin>"
    "<param name=\"robot_type\">j2n6s200</param>"
    "<param name=\"use_fake_device\">true</param>"
    "<param name=\"enable_services\">false</param>" +
    extra_hw_params +
    "</hardware>";

  for (int n = 1; n <= 3; ++n) {
    urdf += arm_joint(n, "0.6283185307");
  }
  for (int n = 4; n <= 6; ++n) {
    urdf += arm_joint(n, "0.8377580410");
  }

  urdf +=
    "<joint name=\"" + prefix + "joint_finger_1\">"
    "<command_interface name=\"position\">"
    "<param name=\"min\">0</param>"
    "<param name=\"max\">1.3963</param>"
    "</command_interface>"
    "<state_interface name=\"position\"/>"
    "<state_interface name=\"velocity\"/>"
    "</joint>"
    "<joint name=\"" + prefix + "joint_finger_2\">"
    "<state_interface name=\"position\"/>"
    "<state_interface name=\"velocity\"/>"
    "</joint>"
    "<joint name=\"" + prefix + "joint_finger_tip_1\">"
    "<state_interface name=\"position\"/>"
    "</joint>";
  if (!drop_last_tip) {
    urdf +=
      "<joint name=\"" + prefix + "joint_finger_tip_2\">"
      "<state_interface name=\"position\"/>"
      "</joint>";
  }

  urdf +=
    "</ros2_control>"
    "</robot>";
  return urdf;
}

#endif  // KINOVA_DRIVER__TEST__KINOVA_TEST_URDF_HPP_
