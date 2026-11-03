"""Integration test: kinova_robot.launch.py with use_fake_hardware:=true.

Brings up robot_state_publisher, ros2_control_node with the KinovaSystem plugin
on its FakeKinovaDevice, and the joint_state_broadcaster, arm_controller and
gripper_controller spawners, then exercises both action servers.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

PREFIX = 'j2n6s200'
ARM_JOINTS = [f'{PREFIX}_joint_{i}' for i in range(1, 7)]
CONTRACT_JOINTS = ARM_JOINTS + [
    f'{PREFIX}_joint_finger_1',
    f'{PREFIX}_joint_finger_2',
    f'{PREFIX}_joint_finger_tip_1',
    f'{PREFIX}_joint_finger_tip_2',
]
CONTROLLERS = {'joint_state_broadcaster', 'arm_controller', 'gripper_controller'}
HOME_JOINT_1 = 4.8046852
READY_TIMEOUT_S = 30.0


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('kinova_bringup'), 'launch', 'kinova_robot.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'use_fake_hardware': 'true'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class TestFakeBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_fake_bringup')
        cls.first_joint_state = None
        cls.latest_joint_state = None
        cls.recording = None
        cls.node.create_subscription(JointState, '/joint_states', cls._on_joint_state, 50)
        cls.list_client = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        cls.arm_client = ActionClient(
            cls.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        cls.gripper_client = ActionClient(
            cls.node, GripperCommand, '/gripper_controller/gripper_cmd')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        if cls.first_joint_state is None:
            cls.first_joint_state = msg
        cls.latest_joint_state = msg
        if cls.recording is not None:
            cls.recording.append(msg)

    # ---------------------------------------------------------------- helpers

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def spin_until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'future did not complete in time')
        return future.result()

    def controller_states(self):
        if not self.list_client.wait_for_service(timeout_sec=1.0):
            return {}
        future = self.list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return {}
        return {c.name: c.state for c in future.result().controller}

    def wait_ready(self):
        """Block until all controllers are active and /joint_states is flowing.

        The controller_manager services are only queried once both action servers
        exist, i.e. after the spawners have activated arm_controller and
        gripper_controller. Polling list_controllers while the spawners are still
        talking to a just-started controller_manager crashed ros2_control_node in
        Humble (rmw_fastrtps "failed to send response: cannot publish data",
        an uncaught RCLError in the service callback) in 2 of 3 runs.
        """
        deadline = time.monotonic() + READY_TIMEOUT_S

        def remaining():
            return max(0.0, deadline - time.monotonic())

        self.assertTrue(self.arm_client.wait_for_server(timeout_sec=remaining()),
                        f'arm action server not available within {READY_TIMEOUT_S} s')
        self.assertTrue(self.gripper_client.wait_for_server(timeout_sec=remaining()),
                        f'gripper action server not available within {READY_TIMEOUT_S} s')
        self.assertTrue(
            self.spin_until(lambda: self.latest_joint_state is not None, remaining()),
            'no /joint_states received')
        states = {}

        def ready():
            nonlocal states
            states = self.controller_states()
            return all(states.get(c) == 'active' for c in CONTROLLERS)

        self.assertTrue(
            self.spin_until(ready, remaining()),
            f'controllers not all active within {READY_TIMEOUT_S} s: {states}')

    def fresh_positions(self):
        """Positions from a /joint_states message received after this call."""
        stamp = self.latest_joint_state
        self.spin_until(lambda: self.latest_joint_state is not stamp, 2.0)
        msg = self.latest_joint_state
        return dict(zip(msg.name, msg.position))

    def send_arm_goal(self, points, timeout):
        self.assertTrue(self.arm_client.wait_for_server(timeout_sec=READY_TIMEOUT_S),
                        'arm action server not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = points
        handle = self.wait_future(self.arm_client.send_goal_async(goal), 5.0)
        self.assertTrue(handle.accepted, 'arm goal rejected')
        return self.wait_future(handle.get_result_async(), timeout).result

    @staticmethod
    def point(positions, t):
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in positions]
        p.time_from_start = Duration(seconds=t).to_msg()
        return p

    # ------------------------------------------------------------------ tests
    # Test methods run in alphabetical order; only test_wrapped_start_goal_succeeds
    # moves joint_1, and test_joint_states_published checks the first /joint_states
    # message received (before any goal), so the order does not matter.

    def test_controllers_active(self):
        self.wait_ready()
        states = self.controller_states()
        for c in CONTROLLERS:
            self.assertEqual(states.get(c), 'active', f'{c}: {states}')

    def test_joint_states_published(self):
        self.wait_ready()
        msg = self.first_joint_state
        self.assertEqual(set(CONTRACT_JOINTS), set(msg.name))
        positions = dict(zip(msg.name, msg.position))
        self.assertAlmostEqual(positions[f'{PREFIX}_joint_1'], HOME_JOINT_1, delta=1e-3)

    def test_arm_trajectory_goal_succeeds(self):
        self.wait_ready()
        current = self.fresh_positions()
        start = [current[j] for j in ARM_JOINTS]
        target = list(start)
        target[5] += 0.2
        result = self.send_arm_goal([self.point(target, 2.0)], timeout=15.0)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.error_string)
        final = self.fresh_positions()
        self.assertLess(abs(angle_diff(final[ARM_JOINTS[5]], target[5])), 0.02)

    def test_wrapped_start_goal_succeeds(self):
        """Review Focus 1: a goal whose first point is joint_1 - 2pi must not spin the arm."""
        self.wait_ready()
        current = self.fresh_positions()
        start = [current[j] for j in ARM_JOINTS]
        q1 = start[0]
        p1 = list(start)
        p1[0] = q1 - 2.0 * math.pi
        p2 = list(start)
        p2[0] = q1 - 2.0 * math.pi + 0.2
        type(self).recording = []
        try:
            result = self.send_arm_goal(
                [self.point(p1, 0.5), self.point(p2, 2.5)], timeout=15.0)
        finally:
            recorded = type(self).recording
            type(self).recording = None
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
                         result.error_string)
        self.assertGreater(len(recorded), 10, 'too few /joint_states samples recorded')
        idx = recorded[0].name.index(ARM_JOINTS[0])
        max_speed = max(abs(m.velocity[idx]) for m in recorded)
        self.node.get_logger().info(
            f'wrapped-start goal: joint_1 peak |velocity| = {max_speed:.4f} rad/s '
            f'over {len(recorded)} samples')
        self.assertLess(max_speed, 0.7, f'joint_1 peaked at {max_speed:.3f} rad/s')
        final = self.fresh_positions()
        self.assertLess(abs(angle_diff(final[ARM_JOINTS[0]], q1 + 0.2)), 0.02)

    def test_gripper_goal_succeeds(self):
        self.wait_ready()
        self.assertTrue(self.gripper_client.wait_for_server(timeout_sec=READY_TIMEOUT_S),
                        'gripper action server not available')
        goal = GripperCommand.Goal()
        goal.command.position = 0.698
        goal.command.max_effort = 0.0
        handle = self.wait_future(self.gripper_client.send_goal_async(goal), 5.0)
        self.assertTrue(handle.accepted, 'gripper goal rejected')
        result = self.wait_future(handle.get_result_async(), 15.0).result
        self.assertTrue(result.reached_goal or result.stalled, str(result))
        self.assertAlmostEqual(result.position, 0.698, delta=0.02)
        final = self.fresh_positions()
        self.assertAlmostEqual(final[f'{PREFIX}_joint_finger_1'], 0.698, delta=0.02)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
