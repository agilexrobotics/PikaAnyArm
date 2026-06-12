import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pika_remote_agx_arm_pkg_dir = get_package_share_directory("pika_remote_agx_arm")
    agx_arm_ctrl_pkg_dir = get_package_share_directory("agx_arm_ctrl")

    arm_ik_param_file = os.path.join(
        pika_remote_agx_arm_pkg_dir, "config", "arm_ik_pose_node.piper_x.yaml"
    )

    # 1) ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py ...
    agx_arm_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can0",
            "arm_type": "piper_x",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
        }.items(),
    )
        
    # 2) ros2 run pika_remote_agx_arm arm_ik_pose_node.py --ros-args --params-file ...
    arm_ik_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="arm_ik_pose_node.py",
        name="arm_ik_pose_node",
        output="screen",
        parameters=[arm_ik_param_file],
    )

    # 3) ros2 run pika_remote_agx_arm pub_delta_pose.py
    pub_delta_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="pub_delta_pose.py",
        name="pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "handle_pose_roll": -1.57,
                "handle_pose_pitch": 0.0,
                "handle_pose_yaw": -1.57,
            }
        ],
    )
    
    return LaunchDescription(
        [
            agx_arm_rviz_launch,
            arm_ik_pose_node,
            pub_delta_pose_node,
        ]
    )
