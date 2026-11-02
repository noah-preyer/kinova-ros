#ifndef KINOVA_DRIVER__KINOVA_COMM_CONFIG_HPP_
#define KINOVA_DRIVER__KINOVA_COMM_CONFIG_HPP_

#include <string>

namespace kinova
{

/// Everything KinovaComm used to read from the ROS 1 parameter server.
struct KinovaCommConfig
{
  /// Kinova type code, e.g. "j2n6s200" ([3] = arm DOF, [5] = number of fingers).
  std::string robot_type;
  /// Serial number of the arm to use. "not_set" selects the first device found.
  std::string serial_number = "not_set";
  /// "USB" or "Ethernet".
  std::string connection_type = "USB";
  /// Ethernet only.
  std::string local_machine_ip = "192.168.100.100";
  std::string subnet_mask = "255.255.255.0";
  int local_cmd_port = 25000;
  int local_broadcast_port = 25025;
  /// Use the estimated (optimal) COM parameters instead of the manual ones.
  bool use_estimated_com = false;
  /// Open the fingers fully when connecting.
  bool movement_on_start = true;
  /// Directory holding the vendor .so files. Empty means a bare-name dlopen (LD_LIBRARY_PATH).
  std::string api_library_dir;
};

}  // namespace kinova

#endif  // KINOVA_DRIVER__KINOVA_COMM_CONFIG_HPP_
