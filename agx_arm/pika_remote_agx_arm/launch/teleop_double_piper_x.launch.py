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
    left_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can_left",
            "namespace": "left_arm",
            "arm_type": "piper_x",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
        }.items(),
    )

    right_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can_right",
            "namespace": "right_arm",
            "arm_type": "piper_x",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
        }.items(),
    )

    # 2) ros2 run pika_remote_agx_arm arm_ik_pose_node.py --ros-args --params-file ...
    left_arm_ik_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="arm_ik_pose_node.py",
        name="left_arm_ik_pose_node",
        output="screen",
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/left_delta_pose",
                "feedback_joint_topic": "/left_arm/feedback/joint_states",
                "pin_joint_status_topic": "/left_arm/control/joint_states",
                "fk_pose_topic": "/left_arm/ik_fk_pose",
            },
        ],
    )

    right_arm_ik_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="arm_ik_pose_node.py",
        name="right_arm_ik_pose_node",
        output="screen",
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/right_delta_pose",
                "feedback_joint_topic": "/right_arm/feedback/joint_states",
                "pin_joint_status_topic": "/right_arm/control/joint_states",
                "fk_pose_topic": "/right_arm/ik_fk_pose",
            },
        ],
    )

    # 3) ros2 run pika_remote_agx_arm pub_delta_pose.py
    # Left hand -> left arm delta pose
    left_pub_delta_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="pub_delta_pose.py",
        name="left_pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "hand_name": "left",
                "handle_pose_topic": "/pika_pose_l",
                "feedback_tcp_pose_topic": "/left_arm/feedback/tcp_pose",
                "delta_pose_topic": "/left_delta_pose",
                "control_joint_topic": "/left_arm/control/joint_states",
                "teleop_trigger_service": "/teleop_trigger_r",
                "gripper_joint_state_topic": "/sensor/gripper_l/joint_state",
                "gripper_max_range": 0.09,
                "handle_pose_roll": -1.57,
                "handle_pose_pitch": 0.0,
                "handle_pose_yaw": -1.57,
            }
        ],
    )
    
    # Right hand -> right arm delta pose
    right_pub_delta_pose_node = Node(
        package="pika_remote_agx_arm",
        executable="pub_delta_pose.py",
        name="right_pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "hand_name": "right",
                "handle_pose_topic": "/pika_pose_r",
                "feedback_tcp_pose_topic": "/right_arm/feedback/tcp_pose",
                "delta_pose_topic": "/right_delta_pose",
                "control_joint_topic": "/right_arm/control/joint_states",
                "teleop_trigger_service": "/teleop_trigger_r",
                "gripper_joint_state_topic": "/sensor/gripper_r/joint_state",
                "gripper_max_range": 0.09,
                "handle_pose_roll": -1.57,
                "handle_pose_pitch": 0.0,
                "handle_pose_yaw": -1.57,
            }
        ],
    )

    return LaunchDescription(
        [
            left_arm_driver_launch,
            right_arm_driver_launch,
            left_arm_ik_pose_node,
            right_arm_ik_pose_node,
            left_pub_delta_pose_node,
            right_pub_delta_pose_node,
        ]
    )
