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

Wait until the launch log shows the three spawners have loaded and activated their controllers before querying `controller_manager`.

Pass: the log contains `Found 1 device(s), using device at index 0`, and `list_controllers` shows 3 controllers, all `active`.

## 3. Joint angles

```bash
python3 -c "
import rclpy, math
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node('j')
def cb(m):
    print(sorted((k[-7:], round(math.degrees(p), 2)) for k, p in zip(m.name, m.position) if 'joint_' in k and 'finger' not in k)); raise SystemExit
n.create_subscription(JointState, '/joint_states', cb, 10)
try: rclpy.spin(n)
except SystemExit: pass
"
```

Pass: `joint_1` to `joint_6` (printed in deg) match the angles JacoSoft or the Kinova SDK show, within 0.5 deg.

## 4. Velocity scale

`docs/arm_goal.py` reads the current `/joint_states`, sends a `FollowJointTrajectory` goal that moves one joint relative to where it is, and prints the peak reported velocity:

```bash
python3 docs/arm_goal.py --joint 6 --delta 0.35 --duration 2
```

Pass: the peak joint_6 velocity equals Delta position / Delta t (0.35 / 2 = 0.175 rad/s) within 10%. A factor of about 2 means the x2 in `KinovaComm::getJointVelocities` is wrong for this firmware. Stop and report it.

## 5. Fingers during arm motion

Send an arm goal and a gripper goal together:

```bash
python3 docs/arm_goal.py --joint 6 --delta 0.35 --duration 4 &
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7, max_effort: 0.0}}"
```

Pass: both the arm and the fingers move. If the fingers stay still, `ANGULAR_VELOCITY` + `POSITION_MODE` is not honoured by this firmware. Stop and report it.

## 6. Stop mid-motion

```bash
python3 docs/arm_goal.py --joint 6 --delta 0.35 --duration 4 &
sleep 2
ros2 control set_controller_state arm_controller inactive
```

(Without the CLI: `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: [arm_controller], strictness: 2}"`.) Reactivate afterwards with `ros2 control set_controller_state arm_controller active`.

Pass: the arm stops within 0.2 s and does not creep afterwards.

## 7. Home service interlock

With `arm_controller` active:

```bash
ros2 service call /j2n6s200_driver/in/home_arm kinova_msgs/srv/HomeArm
```

Pass: the result starts with `Rejected`. After `ros2 control set_controller_state arm_controller inactive`, the same call homes the arm. Wait for homing to finish (arm at rest) before reactivating `arm_controller`.

Then, with `arm_controller` active, call `in/stop` and then `in/start`:

```bash
ros2 service call /j2n6s200_driver/in/stop kinova_msgs/srv/Stop
ros2 service call /j2n6s200_driver/in/start kinova_msgs/srv/Start
```

Pass: `start` returns `Rejected: deactivate arm_controller before starting` and the arm does not move. Deactivate `arm_controller`, call `in/start` again (`Arm started`), then reactivate `arm_controller`: the arm stays where it is.

## 8. Grasp stall

Put an object about 3 cm wide between the fingers and close the gripper on it:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 1.2, max_effort: 0.0}}"
```

Pass: the result has `stalled: true`, the goal is not aborted, and the arm itself does not move.

## 9. MoveIt

Stop the previous launch (it is started again by this one), then:

```bash
ros2 launch j2n6s200_moveit_config move_group.launch.py
```

Pass: in RViz, plan and execute arm `Home` -> `Vertical` -> `Home`, and gripper `Open` / `Close`.

## 10. USB unplug

Unplug USB while the driver is idle.

Pass: `read` logs an error and the process does not crash. Re-plug and restart the launch: normal operation resumes.

## 11. Ctrl-C mid-motion

```bash
python3 docs/arm_goal.py --joint 6 --delta 0.35 --duration 4 &
sleep 2
```

Then press Ctrl-C in the terminal running `kinova_robot.launch.py`.

Pass: the arm stops within 0.2 s and does not creep afterwards.

## Without hardware

`ros2 launch kinova_bringup kinova_robot.launch.py use_fake_hardware:=true` runs the same stack on a simulated device, useful to rehearse checks 2, 4, 5, 6, 7 and 9 (not 1, 3, 8, 10, 11).
