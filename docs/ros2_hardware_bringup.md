# ROS 2 hardware bring-up checklist (j2n6s200)

For the person with the arm. Run the checks in order in a sourced workspace (`source install/setup.bash`). Where a check says "stop and report", do not continue: the driver has an assumption that this firmware may not meet. Keep the arm clear of obstacles and a hand near the power switch.

`ros2 control` comes from `ros-humble-ros2controlcli`. If it is not installed, use the `/controller_manager/list_controllers` and `/controller_manager/switch_controller` services (`switch_controller` with `{deactivate_controllers: [arm_controller], strictness: 2}`, as in the README).

Wait for the launch to finish starting before running `ros2 control` commands (Humble's `controller_manager` can abort if queried during startup).

## 1. USB permissions

```bash
sudo cp kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/ && sudo udevadm control --reload
lsusb | grep 22cd
```

Pass: exactly one device is listed (re-plug the arm if it was already connected).

## 2. Driver starts

```bash
ros2 launch kinova_bringup kinova_robot.launch.py
ros2 control list_controllers      # second terminal
```

Pass: the log contains `Found 1 device(s), using device at index 0`, and `list_controllers` shows 3 controllers, all `active`.

## 3. Joint angles

```bash
ros2 topic echo /joint_states
```

Pass: `j2n6s200_joint_1` to `_6` (rad, convert to deg) match the angles JacoSoft or the Kinova SDK show, within 0.5 deg.

## 4. Velocity scale

Send a `FollowJointTrajectory` goal moving `j2n6s200_joint_6` by +0.35 rad over 2 s, for example:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [j2n6s200_joint_1, j2n6s200_joint_2, j2n6s200_joint_3, j2n6s200_joint_4, j2n6s200_joint_5, j2n6s200_joint_6], points: [{positions: [<current j1..j5>, <current j6 + 0.35>], time_from_start: {sec: 2}}]}}"
```

Fill the positions from `/joint_states` (all joints hold their current value except joint_6). Record `/joint_states` during the move.

Pass: peak reported `velocity` of joint_6 equals Delta position / Delta t within 10%. A factor of about 2 means the x2 in `KinovaComm::getJointVelocities` is wrong for this firmware. Stop and report it.

## 5. Fingers during arm motion

Send an arm goal (as in check 4) and a gripper goal together:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7, max_effort: 0.0}}"
```

Pass: both the arm and the fingers move. If the fingers stay still, `ANGULAR_VELOCITY` + `POSITION_MODE` is not honoured by this firmware. Stop and report it.

## 6. Stop mid-motion

Start a 4 s arm move, then:

```bash
ros2 control set_controller_state arm_controller inactive
```

Pass: the arm stops within 0.2 s and does not creep afterwards.

## 7. Home service interlock

With `arm_controller` active:

```bash
ros2 service call /j2n6s200_driver/in/home_arm kinova_msgs/srv/HomeArm
```

Pass: the result starts with `Rejected`. After `ros2 control set_controller_state arm_controller inactive`, the same call homes the arm.

## 8. Grasp stall

Close the gripper on an object about 3 cm wide (`gripper_cmd` as in check 5 with a closing position).

Pass: the result has `stalled: true` and the goal is not aborted.

## 9. MoveIt

Stop the previous launch (it is started again by this one), then:

```bash
ros2 launch j2n6s200_moveit_config move_group.launch.py
```

Pass: in RViz, plan and execute arm `Home` -> `Vertical` -> `Home`, and gripper `Open` / `Close`.

## 10. USB unplug

Unplug USB while the driver is idle.

Pass: `read` logs an error and the process does not crash. Re-plug and restart the launch: normal operation resumes.

## Without hardware

`ros2 launch kinova_bringup kinova_robot.launch.py use_fake_hardware:=true` runs the same stack on a simulated device, useful to rehearse checks 2, 4, 5, 6, 7 and 9 (not 1, 3, 8, 10).
