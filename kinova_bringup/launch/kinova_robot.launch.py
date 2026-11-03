"""Bring up a Kinova arm with ros2_control.

Starts robot_state_publisher, the controller_manager (ros2_control_node) with the
KinovaSystem hardware interface, and spawners for joint_state_broadcaster,
arm_controller and gripper_controller.

Launch arguments:
  robot_type        Kinova type code (default j2n6s200). Selects
                    kinova_description/urdf/<robot_type>_standalone.xacro and
                    kinova_bringup/config/<robot_type>_controllers.yaml.
  use_fake_hardware true: simulated FakeKinovaDevice, no arm needed (default false).
  serial_number     serial of the arm to use; not_set picks the first (default not_set).
  connection_type   USB or Ethernet (default USB).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    serial_number = LaunchConfiguration('serial_number')
    connection_type = LaunchConfiguration('connection_type')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_type', default_value='j2n6s200',
            description='Kinova robot type code, e.g. j2n6s200.'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Use the simulated FakeKinovaDevice instead of a real arm.'),
        DeclareLaunchArgument(
            'serial_number', default_value='not_set',
            description='Serial number of the arm to connect to; not_set = first found.'),
        DeclareLaunchArgument(
            'connection_type', default_value='USB',
            description='USB or Ethernet.'),
    ]

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('kinova_description'), 'urdf',
            [robot_type, '_standalone.xacro']]), ' ',
        'use_fake_hardware:=', use_fake_hardware, ' ',
        'serial_number:=', serial_number, ' ',
        'connection_type:=', connection_type,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controllers_file = PathJoinSubstitution([
        FindPackageShare('kinova_bringup'), 'config', [robot_type, '_controllers.yaml']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_file],
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[name, '--controller-manager', '/controller_manager'],
        )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        control_node,
        spawner('joint_state_broadcaster'),
        spawner('arm_controller'),
        spawner('gripper_controller'),
    ])
