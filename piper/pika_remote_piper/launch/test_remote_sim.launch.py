import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1.获取所需包的共享目录
    pika_locator_pkg_dir = get_package_share_directory('pika_locator')

    # 2. pika_single_locator.launch 转换
    pika_locator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pika_locator_pkg_dir, 'launch', 'pika_single_locator.launch.py')
        )
    )

    # ---1. serial_gripper_imu 节点配置---
    # ROS2 中通常不再需要 respawn=true,除非明确配置 LifecycleNode
    serial_gripper_imu_node = Node(
        package='sensor_tools',
        executable='serial_gripper_imu',
        name='serial_gripper_imu_node',
        output='screen',
        parameters=[
            {'serial_port':'/dev/ttyUSB0'},
            {'ctrl_mode':'teleop'},
        ]
    )

    test_piper_sim_node = Node(
        package='pika_remote_piper',
        executable='test_piper_sim.py',
        name='test_piper_sim_node',
        output='screen',
        #无参数传入
    )

    # 返回启动描述
    return LaunchDescription(
        [
            pika_locator_launch,
            serial_gripper_imu_node,
            test_piper_sim_node,
        ]
    )