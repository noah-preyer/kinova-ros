"""Start MoveIt 2 move_group (and optionally RViz) for the Kinova j2n6s200.

Includes kinova_bringup/kinova_robot.launch.py, which starts robot_state_publisher,
ros2_control_node and the joint_state_broadcaster, arm_controller and
gripper_controller spawners.

Launch arguments:
  use_fake_hardware  true: simulated FakeKinovaDevice, no arm needed (default false).
  launch_rviz        start RViz with the MotionPlanning display (default true).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')

    description_share = get_package_share_directory('kinova_description')
    bringup_share = get_package_share_directory('kinova_bringup')
    config_share = get_package_share_directory('j2n6s200_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('j2n6s200', package_name='j2n6s200_moveit_config')
        .robot_description(
            file_path=os.path.join(description_share, 'urdf', 'j2n6s200_standalone.xacro'),
            mappings={'use_fake_hardware': use_fake_hardware})
        .robot_description_semantic('config/j2n6s200.srdf')
        .robot_description_kinematics('config/kinematics.yaml')
        .joint_limits('config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'kinova_robot.launch.py')),
        launch_arguments={
            'robot_type': 'j2n6s200',
            'use_fake_hardware': use_fake_hardware,
        }.items(),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(config_share, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Use the simulated FakeKinovaDevice instead of a real arm.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Start RViz with the MotionPlanning display.'),
        robot,
        move_group,
        rviz,
    ])
