"""Integration test: move_group.launch.py on fake hardware plans and executes named states."""

import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

PACKAGE = 'j2n6s200_moveit_config'
READY_TIMEOUT_S = 60.0
TOLERANCE = 0.01


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory(PACKAGE), 'launch', 'move_group.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'use_fake_hardware': 'true', 'launch_rviz': 'false'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def named_state(name):
    """Joint values of an SRDF group_state, read from the installed SRDF."""
    srdf = ET.parse(os.path.join(
        get_package_share_directory(PACKAGE), 'config', 'j2n6s200.srdf')).getroot()
    for s in srdf.findall('group_state'):
        if s.get('name') == name:
            return s.get('group'), {j.get('name'): float(j.get('value'))
                                    for j in s.findall('joint')}
    raise KeyError(name)


class TestMoveGroupFake(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_move_group_fake')
        cls.latest_joint_state = None
        cls.node.create_subscription(JointState, '/joint_states', cls._on_joint_state, 50)
        cls.arm_client = ActionClient(
            cls.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        cls.gripper_client = ActionClient(
            cls.node, GripperCommand, '/gripper_controller/gripper_cmd')
        cls.plan_client = cls.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        cls.execute_client = ActionClient(cls.node, ExecuteTrajectory, '/execute_trajectory')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.latest_joint_state = msg

    def wait_future(self, future, timeout):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'future did not complete in time')
        return future.result()

    def wait_ready(self):
        """Wait for the controllers' action servers, then for move_group's interfaces.

        No controller_manager service is called: querying it while the spawners are
        still talking to it can crash ros2_control_node on Humble (see kinova_bringup).
        """
        deadline = time.monotonic() + READY_TIMEOUT_S

        def remaining():
            return max(0.0, deadline - time.monotonic())

        for client, name in [(self.arm_client, 'arm controller action'),
                             (self.gripper_client, 'gripper controller action'),
                             (self.plan_client, '/plan_kinematic_path'),
                             (self.execute_client, '/execute_trajectory')]:
            if isinstance(client, ActionClient):
                ok = client.wait_for_server(timeout_sec=remaining())
            else:
                ok = client.wait_for_service(timeout_sec=remaining())
            self.assertTrue(ok, f'{name} not available within {READY_TIMEOUT_S} s')
        end = time.monotonic() + remaining()
        while self.latest_joint_state is None and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNotNone(self.latest_joint_state, 'no /joint_states received')

    def plan_and_execute(self, state_name):
        self.wait_ready()
        group, values = named_state(state_name)
        request = GetMotionPlan.Request()
        req = request.motion_plan_request
        req.group_name = group
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.start_state.is_diff = True
        constraints = Constraints()
        for name, value in values.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = TOLERANCE
            jc.tolerance_below = TOLERANCE
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        response = self.wait_future(self.plan_client.call_async(request), 30.0)
        plan = response.motion_plan_response
        self.assertEqual(plan.error_code.val, MoveItErrorCodes.SUCCESS,
                         f'planning {group} -> {state_name} failed: {plan.error_code.val}')

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan.trajectory
        handle = self.wait_future(self.execute_client.send_goal_async(goal), 10.0)
        self.assertTrue(handle.accepted, 'execute_trajectory goal rejected')
        result = self.wait_future(handle.get_result_async(), 60.0).result
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS,
                         f'executing {group} -> {state_name} failed: {result.error_code.val}')

    def test_plan_and_execute_vertical(self):
        self.plan_and_execute('Vertical')

    def test_plan_and_execute_gripper_close(self):
        self.plan_and_execute('Close')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Every process exits cleanly, except move_group may segfault while exiting.

        MoveIt 2.5.9 (Humble) TrajectoryExecutionManager destroys its controller
        manager plugin loader (unloading libmoveit_simple_controller_manager.so)
        before controller_mgr_node_, whose callback groups still hold weak_ptrs to
        the plugin's action clients; ~CallbackGroup then segfaults (-11). This
        happens after SIGINT, once all work is done.
        """
        clean = [0, -2, -15]
        for info in proc_info:
            allowed = clean + [-11] if info.process_name.startswith('move_group') else clean
            self.assertIn(info.returncode, allowed,
                          f'{info.process_name} exited with code {info.returncode}')
