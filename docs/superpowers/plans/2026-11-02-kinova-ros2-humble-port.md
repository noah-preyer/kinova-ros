# kinova-ros → ROS 2 Humble (ros2_control, j2n6s200) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive a Kinova JACO2 6-DOF non-spherical 2-finger arm (`j2n6s200`) from ROS 2 Humble through a `ros2_control` hardware interface, with `joint_trajectory_controller`, `GripperActionController` and MoveIt 2. The stack must also run on a built-in fake device when no arm is attached.

**Architecture:** The Kinova C++ API wrappers (`kinova_api`, `kinova_comm`) are kept and stripped of ROS 1. `KinovaComm` implements a small `KinovaDevice` interface. A `hardware_interface::SystemInterface` plugin (`kinova_driver/KinovaSystem`) owns one `KinovaDevice`: the real `KinovaComm`, or a `FakeKinovaDevice` that integrates velocity. The arm is commanded in **joint velocity** (the firmware's 100 Hz `ANGULAR_VELOCITY` stream) and closed in position by JTC's PID. The fingers take position commands, folded into the same trajectory point. The old topic API, action servers and Gazebo are not ported in this plan.

**Tech Stack:** ROS 2 Humble, ros2_control / ros2_controllers 2.x (`hardware_interface` 2.54), MoveIt 2.5.9, `moveit_configs_utils`, xacro, ament_cmake, gtest, pytest, launch_testing. The Kinova vendor `.so` libraries are x86_64 only.

**Spec:** No separate spec doc. The requirements are the user's request ("port this repo from ROS to ROS 2 Humble using a ros2_control interface; j2n6s200 is the current target") plus the **Requirements** section below.

## Requirements

1. Humble-buildable colcon workspace. Every package that is not ported is invisible to colcon.
2. `ros2_control` hardware plugin for j2n6s200: arm joints 1–6 and both fingers, real hardware over USB (Ethernet config passed through, untested).
3. Controllers: `joint_state_broadcaster`, `arm_controller` (JTC), `gripper_controller` (GripperActionController).
4. A MoveIt 2 config for j2n6s200. None exists in the ROS 1 tree, so it is derived from `j2n6s300_moveit_config`.
5. A fake-hardware mode that exercises the same plugin code, so everything above is testable without an arm.
6. Minimal driver services kept from ROS 1: `in/home_arm`, `in/start`, `in/stop`.

## Global Constraints

- **Humble shell** (all build/test commands run inside it, from `/ws`):
  `docker run --rm -it -v "$PWD":/ws/src/kinova-ros -v kinova_build:/ws/build -v kinova_install:/ws/install -v kinova_log:/ws/log -w /ws moveit/moveit2:humble-release bash`
  then `source /opt/ros/humble/setup.bash` (and `source install/setup.bash` after building). Run `docker run` from the repo root. The image already contains ros2_control 2.54, ros2_controllers, MoveIt 2.5.9, xacro, launch_testing and gtest. In the Claude cloud sandbox, start the daemon first with `dockerd >/tmp/dockerd.log 2>&1 &`. `packages.ros.org` is blocked there, so do not `apt`/`rosdep` install ROS packages; the image has everything this plan needs.
- **Test command pattern:** `colcon build --packages-up-to <pkg> && colcon test --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose`. Success means the last line reads `Summary: N tests, 0 errors, 0 failures, 0 skipped`.
- C++17. Package manifests use `<package format="3">`. Build type `ament_cmake` throughout.
- New ros2_control code goes in namespace `kinova_driver`. Ported vendor wrappers stay in namespace `kinova`.
- Plugin type string: `kinova_driver/KinovaSystem`.
- Robot prefix and type: `j2n6s200` (8-char Kinova type code; `[3]` = arm DOF = 6, `[5]` = fingers = 2).
- **Units:** the Kinova API uses degrees, deg/s, N·m, and finger "turns" 0…6800. ROS uses rad and rad/s. Finger turns→deg factor is `80.0/6800.0` (ROS 1 `finger_conv_ratio_`). Reported joint velocities keep the ROS 1 handling: `KinovaComm::getJointVelocities` doubles the raw value, then values > 180 have 360 subtracted, then deg→rad.
- **Control loop:** `controller_manager` `update_rate: 100`. The DSP runs a 10 ms loop, and velocity commands must stream at 100 Hz.
- **Safety invariants:** the plugin never streams arm velocity unless all arm velocity interfaces are claimed. It sends exactly one zero-velocity point when they are released. Non-finite commands become 0. Velocity commands are clamped to the URDF velocity limits.
- Vendor libraries exist only for `x86_64-linux-gnu`. Building on any other architecture must succeed with a CMake `WARNING`, and fake mode must still work.
- Packages not ported get an empty `COLCON_IGNORE` file. They are not deleted.
- Commit messages: `<type>(<package>): <summary>`, e.g. `feat(kinova_driver): add KinovaSystem hardware interface`.

### ros2_control interface contract (used by Tasks 4, 6, 7, 8)

`<ros2_control name="j2n6s200_system" type="system">` lists the joints in this order:

| Joint | Command interfaces | State interfaces | Command `min`/`max` params |
|---|---|---|---|
| `j2n6s200_joint_1` … `_3` | `velocity` | `position`, `velocity`, `effort` | `-${joint_N_velocity_limit}` / `${joint_N_velocity_limit}` (= ±0.6283) |
| `j2n6s200_joint_4` … `_6` | `velocity` | `position`, `velocity`, `effort` | same expression (= ±0.8378) |
| `j2n6s200_joint_finger_1` | `position` | `position`, `velocity` | `0` / `1.3963` |
| `j2n6s200_joint_finger_2` | — | `position`, `velocity` | — |
| `j2n6s200_joint_finger_tip_1`, `_2` | — | `position` | — |

Hardware `<param>`s (all strings; default used when absent):

| Param | Default | Meaning |
|---|---|---|
| `robot_type` | *(required)* | Kinova type code, validated by `kinova::valid_kinovaRobotType` |
| `use_fake_device` | `false` | `true` → `FakeKinovaDevice` |
| `serial_number` | `not_set` | pick arm by serial; `not_set` = first device |
| `connection_type` | `USB` | `USB` or `Ethernet` |
| `local_machine_ip` / `subnet_mask` / `local_cmd_port` / `local_broadcast_port` | `192.168.100.100` / `255.255.255.0` / `25000` / `25025` | Ethernet only |
| `finger_max_turn` | `6800` | clamp for finger commands |
| `init_fingers_on_configure` | `true` | ROS 1 `is_movement_on_start` (fingers open fully on connect) |
| `enable_services` | `true` | start the `in/home_arm`, `in/start`, `in/stop` node |

## Review Focus

1. **Continuous joint reported above π.** Fake home has `joint_1` = 4.8047 rad. MoveIt's `FixStartStateBounds` sends trajectories that start at `4.8047 − 2π`. The arm must not spin a full turn, and the goal must succeed. Covered by Task 7 `test_wrapped_start_goal_succeeds` and Task 4 `ReadUnwrapsPositionJump`.
2. **Controller deactivated or crashed mid-motion.** The last velocity must not persist: one zero point, then silence. Covered by Task 4 `StopSwitchZeroesArmVelocity`.
3. **Garbage velocity commands.** A NaN from an unset interface, or a huge PID output, must become 0 or be clamped to the URDF limit. Covered by Task 4 `WriteClampsAndSanitizesVelocity`.
4. **Vendor libraries missing.** This happens on aarch64, with an unsourced workspace, or with a wrong library dir. Configure must fail with a clear error and no segfault, and fake mode must still work. Covered by Task 2 `InitializeFailsWhenLibrariesMissing` and Task 4, which runs entirely on the fake.
5. **`home_arm` while JTC holds the arm.** Firmware homing would fight JTC's hold, and JTC would drive the arm back afterwards, so the service must reject. `stop` must silence `write()`. Covered by Task 5 `HomeRejectedWhileArmClaimed` and `WriteSilentAfterStopService`.

---

## File Structure

| Path | Responsibility |
|---|---|
| `docker/Dockerfile.humble` | Reproducible Humble build/run image (base + `libusb-1.0-0`) |
| `kinova_msgs/` | ported rosidl interfaces (renamed fields/files) |
| `kinova_driver/include/kinova_driver/kinova_comm_config.hpp` | plain config struct replacing `ros::NodeHandle` params |
| `kinova_driver/{include,src}/…/kinova_api.*`, `kinova_comm.*`, `kinova_ros_types.*` | vendor wrapper, ROS-1-free |
| `kinova_driver/include/kinova_driver/conversions.hpp` | pure unit conversions (header-only) |
| `kinova_driver/include/kinova_driver/kinova_device.hpp` | device seam used by the plugin |
| `kinova_driver/{include,src}/…/fake_kinova_device.*` | simulated DSP for tests and `use_fake_device` |
| `kinova_driver/{include,src}/…/kinova_system.*`, `kinova_driver_plugins.xml` | ros2_control `SystemInterface` + services |
| `kinova_driver/test/*` | gtests |
| `kinova_description/urdf/kinova.ros2_control.xacro` | `<ros2_control>` macro (contract above) |
| `kinova_description/urdf/j2n6s200_standalone.xacro` | adds xacro args + macro call |
| `kinova_description/test/test_j2n6s200_xacro.py` | renders xacro, checks ros2_control block |
| `kinova_bringup/launch/kinova_robot.launch.py`, `config/j2n6s200_controllers.yaml` | driver + controllers bringup |
| `kinova_bringup/test/test_fake_bringup.py` | launch_testing with fake device |
| `kinova_moveit/robot_configs/j2n6s200_moveit_config/` | new MoveIt 2 config package |
| `README.md`, `docs/ros2_hardware_bringup.md` | usage + human hardware checklist |

---

### Task 1: Build image, workspace triage, `kinova_msgs` port

**Files:**
- Create: `docker/Dockerfile.humble`
- Create (empty): `COLCON_IGNORE` in `kinova_control/`, `kinova_demo/`, `kinova_gazebo/`, `kinova_moveit/inverse_kinematics_plugins/`, `kinova_moveit/kinova_arm_moveit_demo/`, and in each of `kinova_moveit/robot_configs/{j2n6s300,j2s6s300,j2s7s300,m1n6s300}_moveit_config/`
- Modify: `kinova_msgs/package.xml`, `kinova_msgs/CMakeLists.txt`, `kinova_msgs/msg/KinovaPose.msg`, `kinova_msgs/srv/AddPoseToCartesianTrajectory.srv`
- Rename: `kinova_msgs/action/Arm_KinovaPose.action` → `kinova_msgs/action/ArmKinovaPose.action` (rosidl type names must match `^[A-Z][A-Za-z0-9]*$`)

**Interfaces:**
- Produces: `kinova_msgs/msg/{CartesianForce,FingerPosition,JointAngles,JointTorque,JointVelocity,KinovaPose,PoseVelocity,PoseVelocityWithFingerVelocity,PoseVelocityWithFingers}`, `srv/{AddPoseToCartesianTrajectory,ClearTrajectories,HomeArm,RunCOMParametersEstimation,SetEndEffectorOffset,SetForceControlParams,SetNullSpaceModeState,SetTorqueControlMode,SetTorqueControlParameters,Start,Stop,ZeroTorques}`, `action/{ArmJointAngles,ArmPose,ArmKinovaPose,SetFingersPosition}`. `KinovaPose` and `AddPoseToCartesianTrajectory` request fields become `x y z theta_x theta_y theta_z`, because rosidl rejects upper-case field names. All other fields are unchanged.

- [ ] **Step 1: Write `docker/Dockerfile.humble`.** `FROM moveit/moveit2:humble-release`, then `apt-get install -y libusb-1.0-0` from the Ubuntu archive (needed by `USBCommLayerUbuntu.so` at runtime), `WORKDIR /ws`.
- [ ] **Step 2: Add the `COLCON_IGNORE` files.**
- [ ] **Step 3: Run the build to see it fail.** Run `colcon build --packages-select kinova_msgs`. Expected: FAIL, because the package is catkin-type and has no ament build.
- [ ] **Step 4: Port `kinova_msgs`.** `package.xml`: `buildtool_depend ament_cmake rosidl_default_generators`, `depend geometry_msgs`, `exec_depend rosidl_default_runtime`, `member_of_group rosidl_interface_packages`. CMake: one `rosidl_generate_interfaces(${PROJECT_NAME} <all 25 files> DEPENDENCIES geometry_msgs)`. No `actionlib_msgs`. Apply the renames above.
- [ ] **Step 5: Verify.** Run `colcon build --packages-select kinova_msgs && source install/setup.bash && ros2 interface list | grep -c '^ *kinova_msgs/' && ros2 interface show kinova_msgs/msg/KinovaPose`. Expected: count `25`, and the `KinovaPose` body is exactly `float32 x`, `float32 y`, `float32 z`, `float32 theta_x`, `float32 theta_y`, `float32 theta_z`. Also check that `colcon list --names-only` contains none of the ignored packages.
- [ ] **Step 6: Commit.** `git add docker kinova_msgs $(git ls-files -o --exclude-standard | grep COLCON_IGNORE) && git commit -m "build(kinova_msgs): port interfaces to ROS 2 Humble, ignore unported packages"`

---

### Task 2: `kinova_driver` core library without ROS 1

**Files:**
- Modify: `kinova_driver/package.xml` (format 3; `depend`: `rclcpp`, `kinova_msgs`, `geometry_msgs`, `tf2`, `ament_index_cpp`; `test_depend`: `ament_cmake_gtest`)
- Rewrite: `kinova_driver/CMakeLists.txt`
- Create: `kinova_driver/include/kinova_driver/kinova_comm_config.hpp`
- Modify: `include/kinova_driver/kinova_api.h`, `src/kinova_api.cpp`, `include/kinova_driver/kinova_comm.h`, `src/kinova_comm.cpp`, `include/kinova_driver/kinova_ros_types.h`, `src/kinova_ros_types.cpp`
- Delete (superseded by ros2_controllers): `include/kinova_driver/kinova_joint_trajectory_controller.h`, `src/kinova_joint_trajectory_controller.cpp`, `include/kinova_driver/joint_trajectory_action_server.h`, `include/kinova_driver/gripper_command_action_server.h`, `src/joint_trajectory_action/`
- Leave in place, not compiled (listed as "not yet ported" in Task 9): `kinova_arm.*`, `kinova_*_action.*`, `kinova_arm_kinematics.*`, `kinova_tf_updater.h`, `src/nodes/`, `src/ikfast/`
- Test: `kinova_driver/test/test_kinova_api.cpp`, `kinova_driver/test/test_kinova_ros_types.cpp`

**Interfaces:**
- Produces:
  - `struct kinova::KinovaCommConfig { std::string robot_type; std::string serial_number = "not_set"; std::string connection_type = "USB"; std::string local_machine_ip = "192.168.100.100"; std::string subnet_mask = "255.255.255.0"; int local_cmd_port = 25000; int local_broadcast_port = 25025; bool use_estimated_com = false; bool movement_on_start = true; std::string api_library_dir; };`. An empty `api_library_dir` means bare-name `dlopen` (LD_LIBRARY_PATH).
  - `int kinova::KinovaAPI::initializeKinovaAPIFunctions(KinovaAPIType type, const std::string & library_dir = "")`. Returns `1` on success. Returns `0` if either library fails to `dlopen` or any `dlsym` returns null, logging the library or symbol name. This replaces today's `assert`, which compiles out in Release and leaves null function pointers.
  - `explicit kinova::KinovaComm::KinovaComm(const KinovaCommConfig & config)`. It owns a private `std::recursive_mutex`. It throws `KinovaCommException` on any failure, and if `initAPI`/`initEthernetAPI` had already succeeded it calls `closeAPI()` before rethrowing. Every other public method keeps its signature.
  - CMake target `kinova_driver_core` (SHARED, exported via `ament_export_targets`), linked with `dl`.
  - `kinova_ros_types` now uses `tf2::Quaternion`/`tf2::Matrix3x3`, `geometry_msgs::msg::*` and `kinova_msgs::msg::*`. `constructKinovaPoseMsg()` fills the lowercase fields.

- [ ] **Step 1: Write the failing tests.**

```cpp
// test_kinova_api.cpp
TEST(KinovaApi, InitializeFailsWhenLibrariesMissing) {
  kinova::KinovaAPI api;
  EXPECT_EQ(api.initializeKinovaAPIFunctions(kinova::USB, "/nonexistent_dir"), 0);
  EXPECT_EQ(api.initializeKinovaAPIFunctions(kinova::ETHERNET, "/nonexistent_dir"), 0);
}
// test_kinova_ros_types.cpp
TEST(KinovaRosTypes, ValidatesRobotType) {
  EXPECT_TRUE(kinova::valid_kinovaRobotType("j2n6s200"));
  EXPECT_FALSE(kinova::valid_kinovaRobotType("j2n6s20"));
  EXPECT_FALSE(kinova::valid_kinovaRobotType("x2n6s200"));
}
TEST(KinovaRosTypes, AnglesMessageRoundTrip) {
  kinova_msgs::msg::JointAngles msg; msg.joint1 = 10.f; msg.joint6 = 350.f;
  kinova::KinovaAngles angles(msg);
  EXPECT_FLOAT_EQ(angles.Actuator1, 10.f);
  EXPECT_FLOAT_EQ(angles.constructAnglesMsg().joint6, 350.f);
}
TEST(KinovaRosTypes, PoseMessageUsesLowercaseFields) {
  CartesianInfo info{}; info.X = 0.1f; info.ThetaZ = 0.5f;
  auto msg = kinova::KinovaPose(info).constructKinovaPoseMsg();
  EXPECT_FLOAT_EQ(msg.x, 0.1f);
  EXPECT_FLOAT_EQ(msg.theta_z, 0.5f);
}
```

- [ ] **Step 2: Run the tests to see them fail.** Use the test command pattern for `kinova_driver`. Expected: FAIL at configure, because the package is still catkin.
- [ ] **Step 3: Port the library.**
  - Swap `ros/ros.h` for `rclcpp/logging.hpp`, using loggers `kinova_api` and `kinova_comm`. Swap `boost::recursive_mutex` for `std::recursive_mutex`, and `ros::Duration(x).sleep()` for `std::this_thread::sleep_for`.
  - `KinovaComm` reads every former `node_handle.param` from `KinovaCommConfig`. The robot IP `192.168.100.11` and port `55000` stay hard-coded as today.
  - CMake: `if(EXISTS ${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})`, install the four vendor `.so` files to `lib/`. Otherwise emit `message(WARNING "Kinova vendor libraries not available for ${CMAKE_LIBRARY_ARCHITECTURE}; only use_fake_device will work")`.
  - Install `include/` and the vendor headers under `include/kinova/`.
- [ ] **Step 4: Run the tests to see them pass.** Expected: `0 errors, 0 failures`.
- [ ] **Step 5: Commit.** `git commit -am "feat(kinova_driver): port Kinova API wrappers to ament/rclcpp"` (also `git add` the new and deleted files).

---

### Task 3: Unit conversions, `KinovaDevice` seam, `FakeKinovaDevice`

**Files:**
- Create: `kinova_driver/include/kinova_driver/conversions.hpp`, `include/kinova_driver/kinova_device.hpp`, `include/kinova_driver/fake_kinova_device.hpp`, `src/fake_kinova_device.cpp`
- Modify: `include/kinova_driver/kinova_comm.h`, `src/kinova_comm.cpp`, `CMakeLists.txt` (add fake to `kinova_driver_core`, add tests)
- Test: `kinova_driver/test/test_conversions.cpp`, `kinova_driver/test/test_fake_kinova_device.cpp`

**Interfaces:**
- Consumes: `kinova::KinovaAngles`, `kinova::FingerAngles`, `AngularInfo`, `KinovaComm` (Task 2).
- Produces:
  - `conversions.hpp`, namespace `kinova_driver`, all `inline`:
    - `constexpr double kFingerTurnsToDeg = 80.0 / 6800.0;`
    - `double degToRad(double)`, `double radToDeg(double)`
    - `double kinovaVelocityToRad(double deg_per_s)`: if `> 180` subtract 360, then convert deg→rad
    - `double fingerTurnsToRad(double turns)`, `double fingerRadToTurns(double rad)`
    - `double unwrapNear(double angle, double reference)`: returns `angle + 2πk` closest to `reference`
  - `kinova_device.hpp`, namespace `kinova`: abstract `class KinovaDevice` with virtual destructor and pure virtuals:
    - `void getJointAngles(KinovaAngles &)` (deg)
    - `void getJointVelocities(KinovaAngles &)` (deg/s, ×2-corrected)
    - `void getJointTorques(KinovaAngles &)` (N·m)
    - `void getFingerPositions(FingerAngles &)` (turns)
    - `void setAngularControl()`
    - `void setJointVelocitiesWithFingerPositions(const AngularInfo & joint_vel_deg_s, const FingerAngles & fingers_turns)`
    - `void setFingerPositions(const FingerAngles &, int timeout = 0, bool push = true)`
    - `void eraseAllTrajectories()`
    - `void homeArm()`, `void startAPI()`, `void stopAPI()`, `bool isStopped()`
  - `class KinovaComm : public KinovaDevice`. The existing methods get `override`, and `KinovaComm` adds `setJointVelocitiesWithFingerPositions`. That method mirrors `setCartesianVelocitiesWithFingers`, with `Position.Type = ANGULAR_VELOCITY`, `Position.Actuators = joint_vel`, `Position.HandMode = POSITION_MODE`, `Position.Fingers = fingers`, sent via `sendAdvanceTrajectory`, and it returns early when `isStopped()`.
  - `class kinova::FakeKinovaDevice : public KinovaDevice`:
    - Constructor: `explicit FakeKinovaDevice(std::array<double, 6> initial_angles_deg = kFakeHomeDeg, std::function<double()> now_s = <steady_clock seconds>)`.
    - `kFakeHomeDeg` = `radToDeg` of `{4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233}` (the SRDF "Home" pose).
    - Test accessors: `int velocityCommandCount() const`, `AngularInfo lastVelocityCommand() const`, `int fingerCommandCount() const`, `FingerAngles lastFingerCommand() const`, `int homeCount() const`, `void setJointAnglesDeg(const std::array<double, 6> &)`, `void setFingerTurns(double f1, double f2)`.
    - Semantics, all methods guarded by one `std::mutex`:
      - A velocity command stays in effect for `kCommandTimeoutS = 0.05` s after it is received.
      - Angles integrate the commanded velocity over `[last_advance, min(now, last_cmd + 0.05)]` on every getter and setter call.
      - `getJointVelocities` returns the commanded velocity while within the timeout, else 0.
      - Finger commands (from either setter) apply instantly. Torques are always 0.
      - `homeArm` resets the angles to the initial pose and increments `homeCount`.
      - `stopAPI` sets stopped and zeroes velocity. While stopped, every `set*` is ignored and not counted. `startAPI` clears stopped.

- [ ] **Step 1: Write the failing tests.**

```cpp
// test_conversions.cpp
using namespace kinova_driver;
TEST(Conversions, VelocityHandlesTwoComplementEncoding) {
  EXPECT_NEAR(kinovaVelocityToRad(10.0), 0.174533, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(350.0), -0.174533, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(-5.0), -0.087266, 1e-6);
  EXPECT_NEAR(kinovaVelocityToRad(180.0), M_PI, 1e-9);
}
TEST(Conversions, FingerTurns) {
  EXPECT_NEAR(fingerTurnsToRad(6800.0), 1.396263, 1e-6);
  EXPECT_NEAR(fingerRadToTurns(0.698132), 3400.0, 1e-2);
  EXPECT_NEAR(fingerRadToTurns(fingerTurnsToRad(1234.0)), 1234.0, 1e-9);
}
TEST(Conversions, UnwrapNear) {
  EXPECT_NEAR(unwrapNear(0.1, 2 * M_PI - 0.1), 2 * M_PI + 0.1, 1e-12);
  EXPECT_NEAR(unwrapNear(6.2, 0.05), 6.2 - 2 * M_PI, 1e-12);
  EXPECT_NEAR(unwrapNear(1.0, 1.1), 1.0, 1e-12);
}
// test_fake_kinova_device.cpp  (manual clock: double t = 0; auto clk = [&]{ return t; };)
TEST(FakeKinovaDevice, IntegratesCommandedVelocity)        // cmd Actuator1=10 at t=0; t=0.01; getJointAngles → home1 + 0.1 (1e-9)
TEST(FakeKinovaDevice, StopsMovingAfterCommandTimeout)     // cmd 10 at t=0; t=1.0 → moved exactly 0.5 deg; velocities == 0
TEST(FakeKinovaDevice, FingerCommandAppliesImmediately)    // setJointVelocitiesWithFingerPositions(0-vel, {3400,3400,0}) → getFingerPositions == 3400
TEST(FakeKinovaDevice, IgnoresCommandsWhileStopped)        // stopAPI(); cmd → velocityCommandCount()==0, angles unchanged; startAPI(); cmd → count 1
TEST(FakeKinovaDevice, HomeRestoresInitialPose)            // setJointAnglesDeg(zeros); homeArm(); angles == kFakeHomeDeg; homeCount()==1
```

- [ ] **Step 2: Run the tests to see them fail** (they do not compile yet).
- [ ] **Step 3: Implement** the headers, `FakeKinovaDevice`, and the `KinovaComm` changes described above.
- [ ] **Step 4: Run the tests to see them pass.**
- [ ] **Step 5: Commit.** `feat(kinova_driver): add unit conversions, KinovaDevice seam and fake device`

---

### Task 4: `KinovaSystem` ros2_control hardware interface

**Files:**
- Create: `kinova_driver/include/kinova_driver/kinova_system.hpp`, `kinova_driver/src/kinova_system.cpp`, `kinova_driver/kinova_driver_plugins.xml`
- Modify: `kinova_driver/CMakeLists.txt` (SHARED lib `kinova_system` → `kinova_driver_core`, `hardware_interface`, `pluginlib`, `rclcpp_lifecycle`; `pluginlib_export_plugin_description_file(hardware_interface kinova_driver_plugins.xml)`), `kinova_driver/package.xml` (add those `depend`s)
- Test: `kinova_driver/test/kinova_test_urdf.hpp`, `kinova_driver/test/test_kinova_system.cpp`, `kinova_driver/test/test_load_kinova_system.cpp`

**Interfaces:**
- Consumes: `KinovaDevice`, `FakeKinovaDevice`, `KinovaComm(KinovaCommConfig)`, `conversions.hpp` (Tasks 2–3).
- Produces: `class kinova_driver::KinovaSystem : public hardware_interface::SystemInterface`, with the Humble signatures:
  - `on_init(const HardwareInfo &)`
  - `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown` (`const rclcpp_lifecycle::State &`)
  - `export_state_interfaces()`, `export_command_interfaces()`
  - `prepare_command_mode_switch(const std::vector<std::string> & start, const std::vector<std::string> & stop)`, `perform_command_mode_switch(...)`
  - `read(const rclcpp::Time &, const rclcpp::Duration & period)`, `write(const rclcpp::Time &, const rclcpp::Duration &)`
  - `protected: virtual std::unique_ptr<kinova::KinovaDevice> createDevice();` Tests override this.
  - `protected: std::atomic<bool> arm_velocity_claimed_{false};` Task 5 reads it.
- Test helper: `std::string makeTestUrdf(const std::string & extra_hw_params = "", bool drop_last_tip = false)` builds `<robot name="t"><ros2_control …>` per the interface contract, with `use_fake_device=true` and `enable_services=false`. Velocity `min`/`max` are written as `∓0.6283185307` (joints 1–3) and `∓0.8377580410` (joints 4–6). Parse it with `hardware_interface::parse_control_resources_from_urdf(urdf)[0]` from `hardware_interface/component_parser.hpp`.
- Test fixture: `void configure(std::array<double, 6> initial_deg = kinova::kFakeHomeDeg, const std::string & extra_hw_params = "")` runs `on_init` → `on_configure` (the fake is created with `initial_deg`) → export → `on_activate`. Tests that need specific starting angles pass them here instead of mutating the fake afterwards, because `read()` unwraps relative to the previous sample.

**Behaviour to implement (decided here):**

| Hook | Behaviour |
|---|---|
| `on_init` | Read params with the defaults from the table. Validate `robot_type`. Require `joints.size() == dof + 2*fingers`. Joint *i* < dof must end with `joint_<i+1>` and have command exactly `{velocity}` and state `{position, velocity, effort}`. The next `fingers` joints end with `joint_finger_<k>`: the first has command `{position}`, every finger has state `{position, velocity}`. The last `fingers` joints end with `joint_finger_tip_<k>` with state `{position}`. Parse each velocity command's `min`/`max` (empty → unlimited). Any violation → log the offending joint and return `ERROR`. No device access. |
| `on_configure` | `device_ = createDevice()`. Default: `FakeKinovaDevice` if `use_fake_device`, else `KinovaComm` with `api_library_dir = ament_index_cpp::get_package_prefix("kinova_driver") + "/lib"`. Catch `std::exception` → log `what()`, return `ERROR`. Then do one `read()` so states are finite. |
| `on_activate` | `device_->setAngularControl()`. Arm velocity commands = 0. Finger command = current finger_1 position. `finger_target_turns_` = measured finger_1 turns. |
| `on_deactivate` | Send one zero-velocity point (fingers at `finger_target_turns_`), then `eraseAllTrajectories()`. |
| `on_cleanup` / `on_shutdown` | `device_.reset()` (the `KinovaComm` destructor closes the API). |
| `read` | Positions use `unwrapNear(degToRad(raw), previous)` (the first read uses the raw value). Velocities use `kinovaVelocityToRad`. Effort is N·m as-is. Finger positions use `fingerTurnsToRad`. Finger velocity = finite difference over `period` (0 if `period ≤ 0`). Tips = 0. Exception → log (throttled 1 s) and return `ERROR`. |
| `prepare/perform_command_mode_switch` | `arm_velocity_claimed_` = true only when all `dof` arm velocity interfaces are claimed. On release of any arm interface, set every arm command to 0, set `send_zero_once_ = true` and mark not claimed. Track `finger_claimed_` the same way for `<finger_1>/position`. |
| `write` | If `device_->isStopped()`, do nothing. Map each arm command: non-finite → 0; clamp to [min, max]; rad→deg. If `finger_claimed_` and the command is finite: `finger_target_turns_ = clamp(fingerRadToTurns(cmd), 0, finger_max_turn)`. **(a)** If `arm_velocity_claimed_`: call `setJointVelocitiesWithFingerPositions(vel_deg, {target, target, target if 3 fingers else 0})`. **(b)** Else if `send_zero_once_`: send the same call with zero velocity and clear the flag. **(c)** Else if `finger_claimed_` and `|target − last_sent_finger_turns_| > 1.0`: call `setFingerPositions(fingers)` and record it. **(d)** Otherwise send nothing. Exception → log throttled and return `ERROR`. |

- [ ] **Step 1: Write the failing tests.** `TestableKinovaSystem` overrides `createDevice()` to return a `FakeKinovaDevice` on a manual clock and keeps a raw pointer `fake`. The fixture runs `on_init` → `on_configure` → export → `on_activate`, and holds the command handles by name.

```cpp
TEST(KinovaSystemInit, RejectsWrongJointCount)            // makeTestUrdf("", /*drop_last_tip=*/true) → on_init == ERROR
TEST(KinovaSystemInit, RejectsArmJointWithoutVelocityCommand) // joint_3 command "position" → ERROR
TEST_F(KinovaSystemTest, ExportsExpectedInterfaces)       // 24 state names == contract set; 7 command names == 6×"<j>/velocity" + "j2n6s200_joint_finger_1/position"
TEST_F(KinovaSystemTest, ReadConvertsUnits)               // configure({90,180,90,180,90,180}); fake->setFingerTurns(6800,6800); read → position(joint_1)=M_PI/2, joint_2=M_PI, finger_1=1.396263, tip_1=0
TEST_F(KinovaSystemTest, ReadUnwrapsPositionJump)         // fake joint_1 359° → read; set 1° → read → position(joint_1) ≈ 6.300639 (1e-6)
TEST_F(KinovaSystemTest, WriteSendsNothingWhenArmNotClaimed) // write() → fake->velocityCommandCount()==0, fingerCommandCount()==0
TEST_F(KinovaSystemTest, WriteSendsDegreesWhenClaimed)    // claim 6 velocity; cmd joint_1=0.1 → write → lastVelocityCommand().Actuator1 ≈ 5.729578, count==1
TEST_F(KinovaSystemTest, WriteClampsAndSanitizesVelocity) // claim; joint_1=NaN, joint_2=10.0, joint_4=-10.0 → Actuator1==0, Actuator2≈36.0, Actuator4≈-48.0 (1e-3)
TEST_F(KinovaSystemTest, StopSwitchZeroesArmVelocity)     // claim; joint_1=0.1; write (count 1); release; write → count 2 and all Actuators==0; write → count still 2
TEST_F(KinovaSystemTest, FingerCommandEmbeddedWhileArmClaimed) // claim arm+finger; finger_1 cmd 0.698132 → write → lastVelocityCommand count 1, lastFingerCommand().Finger1≈3400 && Finger2≈3400 (0.5)
TEST_F(KinovaSystemTest, FingerCommandSentAloneWhenArmIdle)    // claim finger only; cmd 0.698132 → write ×2 → fingerCommandCount()==1; cmd 0.8 → write → 2
TEST_F(KinovaSystemTest, FingerCommandClampedToMaxTurn)   // claim finger; cmd 1.51 → write → lastFingerCommand().Finger1 == 6800
TEST_F(KinovaSystemTest, WriteSilentWhileStopped)         // fake->stopAPI(); claim; cmd 0.1; write → velocityCommandCount()==0
// test_load_kinova_system.cpp
TEST(KinovaSystemPlugin, LoadsThroughPluginlib)           // pluginlib::ClassLoader<hardware_interface::SystemInterface>("hardware_interface","hardware_interface::SystemInterface").createSharedInstance("kinova_driver/KinovaSystem") != nullptr
```

- [ ] **Step 2: Run the tests to see them fail.**
- [ ] **Step 3: Implement `KinovaSystem`** per the behaviour table, plus `kinova_driver_plugins.xml` (`<class name="kinova_driver/KinovaSystem" type="kinova_driver::KinovaSystem" base_class_type="hardware_interface::SystemInterface">`) and `PLUGINLIB_EXPORT_CLASS`.
- [ ] **Step 4: Run the tests to see them pass**, including the Task 2–3 tests.
- [ ] **Step 5: Commit.** `feat(kinova_driver): add KinovaSystem ros2_control hardware interface`

---

### Task 5: Driver services (`in/home_arm`, `in/start`, `in/stop`)

**Files:**
- Modify: `kinova_driver/include/kinova_driver/kinova_system.hpp`, `kinova_driver/src/kinova_system.cpp`
- Test: `kinova_driver/test/test_kinova_system_services.cpp`

**Interfaces:**
- Consumes: `arm_velocity_claimed_`, `device_` (Task 4), `kinova_msgs/srv/{HomeArm,Start,Stop}` (Task 1).
- Produces:
  - When `enable_services` is true, `on_configure` creates `rclcpp::Node` `<robot_type>_driver` with services `~/in/home_arm`, `~/in/start` and `~/in/stop`. For j2n6s200 that gives `/j2n6s200_driver/in/home_arm`, the same names as ROS 1. The node is spun by a `SingleThreadedExecutor` on a `std::thread`, which is stopped and joined in `on_cleanup`, `on_shutdown` and the destructor.
  - Public handlers (callable directly in tests):
    - `void handleHomeArm(const std::shared_ptr<kinova_msgs::srv::HomeArm::Request>, std::shared_ptr<kinova_msgs::srv::HomeArm::Response>)`. If `arm_velocity_claimed_`, set `homearm_result = "Rejected: deactivate arm_controller before homing"` and do not call the device. Otherwise call `device_->homeArm()` and set `homearm_result = "KINOVA ARM HAS BEEN RETURNED HOME"`.
    - `handleStop`: calls `device_->stopAPI()` and sets `stop_result = "Arm stopped"`.
    - `handleStart`: calls `device_->startAPI()` and sets `start_result = "Arm started"`.

- [ ] **Step 1: Write the failing tests.** The fixture calls `rclcpp::init` once and uses the Task 4 fixture with `enable_services=true`.

```cpp
TEST_F(KinovaSystemServicesTest, HomeRejectedWhileArmClaimed)   // claim arm → handleHomeArm → fake->homeCount()==0, result starts with "Rejected"
TEST_F(KinovaSystemServicesTest, HomeAcceptedWhenIdle)          // no claim → homeCount()==1, result == "KINOVA ARM HAS BEEN RETURNED HOME"
TEST_F(KinovaSystemServicesTest, WriteSilentAfterStopService)   // claim; handleStop; cmd 0.1; write → velocityCommandCount()==0; handleStart; write → 1
TEST_F(KinovaSystemServicesTest, ServicesAdvertised)            // rclcpp node lists "/j2n6s200_driver/in/home_arm", "/…/in/start", "/…/in/stop" within 2 s
```

- [ ] **Step 2: Run the tests to see them fail.**
- [ ] **Step 3: Implement** the node, the executor thread and the handlers.
- [ ] **Step 4: Run the tests to see them pass.**
- [ ] **Step 5: Commit.** `feat(kinova_driver): expose home/start/stop services from hardware interface`

---

### Task 6: `kinova_description` for ROS 2 with ros2_control

**Files:**
- Modify: `kinova_description/package.xml` (format 3, `ament_cmake`, `exec_depend xacro urdf`, `test_depend ament_cmake_pytest`), `kinova_description/CMakeLists.txt` (install `urdf/`; install `meshes/` matching `*.dae`; `ament_add_pytest_test`)
- Create: `kinova_description/urdf/kinova.ros2_control.xacro`
- Modify: `kinova_description/urdf/j2n6s200_standalone.xacro`
- Test: `kinova_description/test/test_j2n6s200_xacro.py`

**Interfaces:**
- Produces:
  - xacro macro `kinova_ros2_control(prefix robot_type use_fake_hardware serial_number connection_type)`, which emits exactly the interface-contract block. Velocity limits come from the `joint_N_velocity_limit` properties already defined by `j2n6s200.xacro`, so they have a single source.
  - `j2n6s200_standalone.xacro` takes args `use_fake_hardware` (default `true`), `serial_number` (`not_set`) and `connection_type` (`USB`). It includes the macro and calls it with `prefix="j2n6s200" robot_type="j2n6s200"`. Existing Gazebo and transmission tags stay; ROS 2 tooling ignores them.

- [ ] **Step 1: Write the failing test** (pytest; renders with `subprocess.run(["xacro", path, *args])`).

```python
def test_ros2_control_block_matches_contract():      # one <ros2_control>; plugin == "kinova_driver/KinovaSystem"; joint names in contract order (10)
def test_interfaces_per_joint():                     # joint_1..6 cmd ["velocity"], state ["position","velocity","effort"]; finger_1 cmd ["position"]; finger_2 no cmd; tips state ["position"]
def test_velocity_limits():                          # joint_1 velocity max ≈ 0.6283, joint_4 ≈ 0.8378 (abs 1e-3); min == -max
def test_fake_flag_passthrough():                    # default → use_fake_device "true"; use_fake_hardware:=false → "false"; serial_number:=PJ123 → "PJ123"
def test_meshes_resolve():                           # every package://kinova_description/meshes/X.dae exists under kinova_description/meshes/
```

- [ ] **Step 2: Run the test to see it fail.**
- [ ] **Step 3: Implement** the macro, the standalone changes and the ament port.
- [ ] **Step 4: Run the test to see it pass.**
- [ ] **Step 5: Commit.** `feat(kinova_description): port to ament and add ros2_control xacro for j2n6s200`

---

### Task 7: `kinova_bringup`: launch, controllers, fake-hardware integration test

**Files:**
- Modify: `kinova_bringup/package.xml` (format 3; `exec_depend`: `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controllers`, `robot_state_publisher`, `xacro`, `kinova_description`, `kinova_driver`, `launch_ros`; `test_depend`: `launch_testing_ament_cmake`, `launch_testing_ros`, `control_msgs`, `controller_manager_msgs`, `sensor_msgs`, `trajectory_msgs`, `rclpy`), `kinova_bringup/CMakeLists.txt`
- Delete: `kinova_bringup/launch/kinova_robot.launch`, `kinova_bringup/launch/config/` (ROS 1)
- Create: `kinova_bringup/launch/kinova_robot.launch.py`, `kinova_bringup/config/j2n6s200_controllers.yaml`
- Test: `kinova_bringup/test/test_fake_bringup.py`

**Interfaces:**
- Consumes: plugin (Task 4), xacro args (Task 6).
- Produces: launch args `robot_type` (default `j2n6s200`; the config file is `config/<robot_type>_controllers.yaml`), `use_fake_hardware` (`false`), `serial_number` (`not_set`), `connection_type` (`USB`). It starts `robot_state_publisher`, `controller_manager/ros2_control_node` (params: `robot_description` + the controllers yaml; Humble takes the description as a parameter) and spawners for `joint_state_broadcaster`, `arm_controller` and `gripper_controller`. Action servers: `/arm_controller/follow_joint_trajectory`, `/gripper_controller/gripper_cmd`. Later tasks include this launch file.

`config/j2n6s200_controllers.yaml` (exact values):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster: {type: joint_state_broadcaster/JointStateBroadcaster}
    arm_controller: {type: joint_trajectory_controller/JointTrajectoryController}
    gripper_controller: {type: position_controllers/GripperActionController}
arm_controller:
  ros__parameters:
    joints: [j2n6s200_joint_1, j2n6s200_joint_2, j2n6s200_joint_3, j2n6s200_joint_4, j2n6s200_joint_5, j2n6s200_joint_6]
    command_interfaces: [velocity]
    state_interfaces: [position, velocity]
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    constraints:
      stopped_velocity_tolerance: 0.02
      goal_time: 1.0
      # each joint: {trajectory: 0.1, goal: 0.02}
    gains:
      # each joint: {p: 5.0, i: 0.0, d: 0.0, i_clamp: 0.0, ff_velocity_scale: 1.0}
      # angle_wraparound: true for joints 1,4,5,6 (continuous); false for 2,3
gripper_controller:
  ros__parameters:
    joint: j2n6s200_joint_finger_1
    action_monitor_rate: 20.0
    goal_tolerance: 0.02
    allow_stalling: true
    stall_velocity_threshold: 0.001
    stall_timeout: 1.0
```

(Write out the six per-joint entries in full; the comments mark where.) `ff_velocity_scale: 1.0` is required. With the default of 0, a velocity-only JTC is a pure P-loop and lags the trajectory. `angle_wraparound` is the Humble name (`normalize_error` is deprecated). It also makes JTC's current-state → first-point segment take the shortest path (`wraparound_joint` in Humble's `trajectory.cpp`), which is what keeps Review Focus 1 from spinning the arm.

- [ ] **Step 1: Write the failing launch test.** `launch_testing` with `use_fake_hardware:=true`, and `rclpy` clients in the test class.

```python
def test_controllers_active(self):            # /controller_manager/list_controllers → {joint_state_broadcaster, arm_controller, gripper_controller} all state "active" within 30 s
def test_joint_states_published(self):        # /joint_states has all 10 contract joint names; j2n6s200_joint_1 ≈ 4.8046852 (1e-3)
def test_arm_trajectory_goal_succeeds(self):  # FollowJointTrajectory: current positions, joint_6 +0.2 at t=2.0 s → error_code == 0; final joint_6 within 0.02
def test_wrapped_start_goal_succeeds(self):   # Review Focus 1: points at 0.5 s (joint_1 = q1 − 2π) and 2.5 s (q1 − 2π + 0.2), others = current
                                              #   → error_code == 0; max |joint_1 velocity| on /joint_states during run < 0.7;
                                              #   final atan2(sin(d), cos(d)) of (joint_1 − (q1 + 0.2)) within 0.02
def test_gripper_goal_succeeds(self):         # GripperCommand position 0.698 → result.reached_goal or result.stalled; position ≈ 0.698 (0.02)
```

- [ ] **Step 2: Run the test to see it fail** (no launch file yet).
- [ ] **Step 3: Implement** the launch file, the yaml and the CMake install of `launch/` and `config/`, plus `add_launch_test(test/test_fake_bringup.py TIMEOUT 120)`.
- [ ] **Step 4: Run the test to see it pass** (`colcon build --packages-up-to kinova_bringup`, then test `kinova_bringup`). If `test_wrapped_start_goal_succeeds` fails, fix it in the plugin or the gains. Do not loosen the assertion.
- [ ] **Step 5: Commit.** `feat(kinova_bringup): ROS 2 launch with ros2_control controllers for j2n6s200`

---

### Task 8: `j2n6s200_moveit_config` (MoveIt 2)

**Files:**
- Create package `kinova_moveit/robot_configs/j2n6s200_moveit_config/`: `package.xml` (`exec_depend`: `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`, `moveit_simple_controller_manager`, `moveit_configs_utils`, `moveit_ros_visualization`, `rviz2`, `kinova_bringup`, `kinova_description`; `test_depend`: `ament_cmake_pytest`, `launch_testing_ament_cmake`, `moveit_msgs`, `rclpy`), `CMakeLists.txt`, `config/j2n6s200.srdf`, `config/kinematics.yaml`, `config/joint_limits.yaml`, `config/moveit_controllers.yaml`, `config/ompl_planning.yaml`, `config/moveit.rviz`, `launch/move_group.launch.py`
- Test: `test/test_srdf_consistency.py`, `test/test_move_group_fake.py`

**Interfaces:**
- Consumes: `kinova_robot.launch.py` and its args (Task 7), and the controller and action names (Task 7).
- Produces: `ros2 launch j2n6s200_moveit_config move_group.launch.py use_fake_hardware:=<bool> launch_rviz:=<bool, default true>`. Planning groups are `arm` and `gripper`. Named states are `Home`, `Retract`, `Vertical` (arm) and `Open`, `Close` (gripper).

**Decisions:**
- **SRDF.** Generate the collision pairs with `sed 's/j2n6s300/j2n6s200/g' ../j2n6s300_moveit_config/config/j2n6s300.srdf | grep -v -E 'finger_3|finger_tip_3'`, which keeps 56 `disable_collisions` lines. Then hand-edit:
  - Remove the `virtual_joint`. The URDF already roots at `world`, and MoveIt rejects a virtual joint whose child is not the URDF root.
  - `arm` = chain `root` → `j2n6s200_end_effector`.
  - `gripper` = joints `j2n6s200_joint_finger_1` and `j2n6s200_joint_finger_2` **only**. Adding tip links would pull tip joints into gripper trajectories that no controller owns.
  - `end_effector name="end_effector" parent_link="j2n6s200_end_effector" group="gripper" parent_group="arm"`.
  - Arm states take their values from the j2n6s300 SRDF (the arm kinematics are identical; only the hand differs). `Open` = 0.2/0.2 and `Close` = 1.2/1.2 on finger_1 and finger_2.
- `kinematics.yaml`: `arm: {kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin, kinematics_solver_search_resolution: 0.005, kinematics_solver_timeout: 0.05}`. trac_ik is not in the image.
- `joint_limits.yaml`: arm joints `max_velocity: 0.35`, `has_acceleration_limits: true`, `max_acceleration: 1.0`. Fingers `max_velocity: 1.0`, `max_acceleration: 1.0`. Top level `default_velocity_scaling_factor: 1.0`, `default_acceleration_scaling_factor: 1.0`.
- `moveit_controllers.yaml`:
  - `moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager`
  - `moveit_simple_controller_manager.controller_names: [arm_controller, gripper_controller]`
  - `arm_controller`: `{type: FollowJointTrajectory, action_ns: follow_joint_trajectory, default: true, joints: [joint_1..6]}`
  - `gripper_controller`: `{type: GripperCommand, action_ns: gripper_cmd, default: true, joints: [j2n6s200_joint_finger_1, j2n6s200_joint_finger_2]}`
  - finger_1 must come first. Humble's handler commands `controller_joints[0]`, and its `command_joint` param is read from the wrong (global) namespace, so do not rely on it.
- `ompl_planning.yaml`: `planning_plugin: ompl_interface/OMPLPlanner`. `request_adapters` uses the Humble default space-separated list `default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints`. `start_state_max_bounds_error: 0.1`. `arm.default_planner_config: RRTConnectkConfigDefault`, with `planner_configs.RRTConnectkConfigDefault: {type: geometric::RRTConnect, range: 0.0}`.
- `move_group.launch.py`: `MoveItConfigsBuilder("j2n6s200", package_name="j2n6s200_moveit_config")`, with `.robot_description(file_path=<abs path to kinova_description/urdf/j2n6s200_standalone.xacro>, mappings={"use_fake_hardware": …})`, `.robot_description_semantic("config/j2n6s200.srdf")`, `.trajectory_execution(file_path="config/moveit_controllers.yaml")` and `.planning_pipelines(pipelines=["ompl"])`. It includes `kinova_robot.launch.py` (forwarding `use_fake_hardware`), starts `move_group` with `moveit_config.to_dict()`, and starts `rviz2` with `config/moveit.rviz` (Fixed Frame `world`, MotionPlanning display, group `arm`) if `launch_rviz`.

- [ ] **Step 1: Write the failing tests.**

```python
# test_srdf_consistency.py  (xacro → URDF links/joints; parse SRDF)
def test_every_srdf_link_exists_in_urdf():   # all link1/link2/parent_link/chain links ∈ URDF links
def test_collision_pair_count():             # len(disable_collisions) == 56
def test_gripper_group_is_two_finger_joints():  # gripper group joints == {finger_1, finger_2}; no <link> children
def test_named_states_cover_group_joints():  # each group_state sets exactly its group's active joints
def test_no_virtual_joint():
# test_move_group_fake.py  (launch move_group.launch.py use_fake_hardware:=true launch_rviz:=false)
def test_plan_and_execute_vertical(self):    # /plan_kinematic_path (GetMotionPlan) group "arm", joint constraints = Vertical (tol 0.01)
                                             #   → error_code.val == 1; /execute_trajectory with planned trajectory → error_code.val == 1
def test_plan_and_execute_gripper_close(self):  # same for group "gripper" to Close → both == 1
```

- [ ] **Step 2: Run the tests to see them fail.**
- [ ] **Step 3: Create the config package** per the decisions above.
- [ ] **Step 4: Run the tests to see them pass** (`colcon build --packages-up-to j2n6s200_moveit_config`, then test it). `test_plan_and_execute_vertical` starts at joint_1 = 4.80 rad, so it also exercises Review Focus 1 end-to-end through MoveIt.
- [ ] **Step 5: Commit.** `feat(j2n6s200_moveit_config): add MoveIt 2 config for j2n6s200`

---

### Task 9: README and human hardware bring-up checklist

**Files:**
- Modify: `README.md`. Rewrite these sections:
  - "Supported versions": this branch targets ROS 2 Humble.
  - Installation: the colcon workspace plus the udev rule.
  - "How to use": the two launch commands, the services, and the action names.
  - "Ported / not yet ported": not yet ported are `kinova_arm` topic API, the action servers, `kinova_tf_updater`, interactive control, ikfast plugins, `kinova_demo`, `kinova_control`, `kinova_gazebo`, the other arm models, and torque/admittance control.
  - "Breaking changes": the `KinovaPose`/`AddPoseToCartesianTrajectory` field renames and `ArmKinovaPose.action`.
- Create: `docs/ros2_hardware_bringup.md`

**Interfaces:** Consumes everything above. Produces documentation only.

- [ ] **Step 1: Write `docs/ros2_hardware_bringup.md`.** Use these numbered checks. Each has an exact command and a pass criterion, and the person with the arm runs them; an agent cannot.
  1. `sudo cp kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/ && sudo udevadm control --reload` and `lsusb | grep 22cd` → one device.
  2. `ros2 launch kinova_bringup kinova_robot.launch.py` → the log contains `Found 1 device(s), using device at index 0`, and `ros2 control list_controllers` shows 3 controllers `active`.
  3. `/joint_states` joint_1..6 (rad→deg) match the angles JacoSoft or the Kinova SDK show within 0.5°.
  4. **Velocity scale.** Send a JTC goal moving joint_6 by +0.35 rad over 2 s. The peak `/joint_states` velocity must equal Δposition/Δt within 10%. A factor of ~2 means the ×2 in `KinovaComm::getJointVelocities` is wrong for this firmware. Stop and report it.
  5. **Fingers during arm motion.** Send an arm goal and a gripper goal (0.7 rad) together. Both must move. If the fingers stay still, `ANGULAR_VELOCITY`+`POSITION_MODE` is not honoured by this firmware. Stop and report it.
  6. **Stop mid-motion.** `ros2 control set_controller_state arm_controller inactive` during a 4 s move. The arm must stop within 0.2 s and must not creep afterwards.
  7. `ros2 service call /j2n6s200_driver/in/home_arm kinova_msgs/srv/HomeArm` while `arm_controller` is active → "Rejected…". After deactivating it, the arm homes.
  8. **Grasp stall.** Close the gripper on a ~3 cm object. The result has `stalled: true` and the goal is not aborted.
  9. `ros2 launch j2n6s200_moveit_config move_group.launch.py`. In RViz, plan and execute arm `Home` → `Vertical` → `Home` and gripper `Open`/`Close`.
  10. Unplug USB while idle → `read` logs an error and the process does not crash. Re-plug and restart the launch → normal operation.
- [ ] **Step 2: Verify the docs.** Check that every command in the README and the checklist runs in the Humble shell with `use_fake_hardware:=true`; the hardware-only checks are exempt. Then check `grep -n "catkin\|roslaunch\|rosrun\|rostopic\|rosservice" README.md` has no matches outside the "ROS 1 branches" pointer.
- [ ] **Step 3: Commit.** `docs: ROS 2 Humble usage and hardware bring-up checklist`

---

## Out of scope / follow-ups

- **MoveIt Task Constructor.** The `noah-preyer/moveit_task_constructor` fork's `ros2` branch includes `moveit/*.hpp` headers, and MoveIt Humble ships only `.h`, so it will not build on Humble. Use upstream `moveit/moveit_task_constructor`'s `humble` branch for an MTC demo on top of `j2n6s200_moveit_config`.
- Gazebo (Ignition Fortress + `gz_ros2_control`), the other arm types, multi-arm prefixes, the Cartesian/joint topic API and action servers, torque and admittance control, and Ethernet validation on hardware.
- Regenerating the SRDF collision matrix with MoveIt Setup Assistant 2 for the 2-finger hand mesh.
