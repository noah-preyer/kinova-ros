# kinova-ros (ROS 2 Humble port)

ROS 2 support for the Kinova JACO2 arm. This branch ports the `j2n6s200` (Jaco2, 6 DOF, non-spherical wrist, 2 fingers) to **ROS 2 Humble** on top of `ros2_control` and MoveIt 2.

This repository doesn't support the Gen3 arm. See [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) for Gen3.

## Supported versions

This branch targets **ROS 2 Humble** (Ubuntu 22.04) and the `j2n6s200` arm only. The Kinova vendor libraries ship for `x86_64-linux-gnu` only; on other architectures the packages build with a CMake warning and only fake hardware mode (`use_fake_hardware:=true`) works.

## ROS 1 branches

The ROS 1 stack lives on other branches and is unchanged there. Use one of these for ROS 1 (catkin, roslaunch, rosrun, rostopic, rosservice):

- [noetic-devel / master](https://github.com/Kinovarobotics/kinova-ros) for ROS Noetic
- [melodic-devel](https://github.com/Kinovarobotics/kinova-ros/tree/melodic-devel) for ROS Melodic
- [kinetic-devel](https://github.com/Kinovarobotics/kinova-ros/tree/kinetic-devel) and [indigo-devel](https://github.com/Kinovarobotics/kinova-ros/tree/indigo-devel), no longer maintained

## Packages

Built by colcon:

- `kinova_msgs`: messages, services, actions.
- `kinova_driver`: `ros2_control` hardware plugin `kinova_driver/KinovaSystem` (real arm via the Kinova C++ API, or a built-in fake device) and the `in/home_arm`, `in/start`, `in/stop` services.
- `kinova_description`: URDF/xacro and meshes, with the `ros2_control` tag.
- `kinova_bringup`: launch file and controller configuration.
- `j2n6s200_moveit_config` (`kinova_moveit/robot_configs/j2n6s200_moveit_config`): MoveIt 2 configuration and `move_group` launch.

All other packages contain a `COLCON_IGNORE` file and are not built (see [Not yet ported](#not-yet-ported)).

## Installation

Requirements: Ubuntu 22.04 with ROS 2 Humble, `ros2_control`, `ros2_controllers`, MoveIt 2 and `libusb-1.0-0`. For a ready-made environment, `docker/Dockerfile.humble` builds on `moveit/moveit2:humble-release` and adds libusb.

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone -b <branch-name> https://github.com/Kinovarobotics/kinova-ros.git
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To access the arm over USB, install the udev rule:

```bash
sudo cp src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/
sudo udevadm control --reload
```

## How to use

Start the driver and controllers. Add `use_fake_hardware:=true` to run without an arm.

```bash
ros2 launch kinova_bringup kinova_robot.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `robot_type` | `j2n6s200` | Kinova type code (only `j2n6s200` is supported) |
| `use_fake_hardware` | `false` | simulated device, no arm needed |
| `serial_number` | `not_set` | pick an arm by serial; `not_set` = first device |
| `connection_type` | `USB` | `USB` or `Ethernet` (Ethernet is untested) |

Controllers (spawned by the launch file):

| Controller | Interface |
|---|---|
| `joint_state_broadcaster` | `/joint_states` |
| `arm_controller` | action `/arm_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) |
| `gripper_controller` | action `/gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`) |

Check them with `ros2 control list_controllers` (needs the `ros-humble-ros2controlcli` package; without it, call `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers`).

Services (provided by the hardware plugin):

```bash
ros2 service call /j2n6s200_driver/in/home_arm kinova_msgs/srv/HomeArm
ros2 service call /j2n6s200_driver/in/stop kinova_msgs/srv/Stop
ros2 service call /j2n6s200_driver/in/start kinova_msgs/srv/Start
```

`home_arm` is rejected while `arm_controller` is active. Deactivate it first:

```bash
ros2 control set_controller_state arm_controller inactive
# without ros2controlcli:
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{deactivate_controllers: [arm_controller], strictness: 2}"
```

MoveIt 2 (starts the driver stack with `move_group` and RViz):

```bash
ros2 launch j2n6s200_moveit_config move_group.launch.py
```

Arguments: `use_fake_hardware` (default `false`), `launch_rviz` (default `true`). The MoveIt config defines the planning groups `arm` and `gripper` with named states `Home`, `Retract`, `Vertical` (arm) and `Open`, `Close` (gripper).

Hardware plugin parameters (set in the URDF `<ros2_control>` block via xacro) include `serial_number`, `connection_type`, the Ethernet settings `local_machine_ip`, `subnet_mask`, `local_cmd_port`, `local_broadcast_port`, `finger_max_turn` (default 6800), `init_fingers_on_configure` (default `true`, fingers open fully on connect) and `enable_services` (default `true`).

First bring-up on a real arm: follow [docs/ros2_hardware_bringup.md](docs/ros2_hardware_bringup.md).

## Not yet ported

These ROS 1 features are not available in this branch (their packages are `COLCON_IGNORE`d):

- the `kinova_arm` topic API (joint/Cartesian position and velocity topics, `out/*` topics)
- the action servers (joint angles, Cartesian pose, fingers)
- `kinova_tf_updater` (`use_urdf:=false` D-H frames)
- interactive control
- inverse kinematics (ikfast) plugins
- `kinova_demo`
- `kinova_control` and `kinova_gazebo` (Gazebo)
- the other arm models (j2n6s300, j2s6s300, j2s7s300, m1n6s300) and their MoveIt configs
- torque control, admittance control and force/torque services

## Breaking changes

- `kinova_msgs/msg/KinovaPose` and `kinova_msgs/srv/AddPoseToCartesianTrajectory`: the fields `X Y Z ThetaX ThetaY ThetaZ` are now `x y z theta_x theta_y theta_z` (ROS 2 message naming rules).
- `Arm_KinovaPose.action` is renamed `ArmKinovaPose.action`.

## Known limitations

- Vendor libraries are x86_64 only; other architectures support fake mode only.
- Ethernet connection is untested.
- MoveIt 2.5.9 `move_group` may segfault on shutdown (upstream issue).
- In Humble, `controller_manager` can abort if its services are queried during startup. Wait for the launch to finish before running `ros2 control` commands.
- Some virtualization software (for example VirtualBox) does not hand USB access to the API correctly (error "1015"); Parallels and VMware work.
- After a firmware update the arm's serial number may be reset to "Not set", which breaks multi-arm setups. Reset it afterwards.

## Report a bug

Bugs, issues or suggestions may be sent to support@kinova.ca.
