#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from std_msgs.msg import Header

def pose_msg_to_mat(msg: PoseStamped) -> np.ndarray:
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = Rotation.from_quat(
        [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
    ).as_matrix()
    mat[:3, 3] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    return mat


def mat2xyzquat(matrix: np.ndarray):
    pos = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quat = Rotation.from_matrix(rotation_matrix).as_quat()
    return pos, quat

class RosOperator(Node):
    def __init__(self):
        super().__init__("pub_delta_pose_node")

        # Topic parameters
        self.declare_parameter("handle_pose_topic", "/pika_pose")
        self.declare_parameter("feedback_tcp_pose_topic", "/feedback/tcp_pose")
        self.declare_parameter("delta_pose_topic", "/delta_pose")
        self.declare_parameter("control_joint_topic", "/control/joint_states")
        self.declare_parameter("gripper_joint_state_topic", "/gripper/joint_state")

        # Service parameters
        self.declare_parameter("teleop_trigger_service", "/teleop_trigger")
        
        # Gripper/control parameters
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("gripper_max_range", 0.07)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("hand_name", "right")
        self.declare_parameter("handle_pose_roll", 0.0)
        self.declare_parameter("handle_pose_pitch", 0.0)
        self.declare_parameter("handle_pose_yaw", 0.0)

        handle_pose_topic = str(self.get_parameter("handle_pose_topic").value)
        feedback_tcp_pose_topic = str(self.get_parameter("feedback_tcp_pose_topic").value)
        delta_pose_topic = str(self.get_parameter("delta_pose_topic").value)
        control_joint_topic = str(self.get_parameter("control_joint_topic").value)
        gripper_joint_state_topic = str(self.get_parameter("gripper_joint_state_topic").value)

        teleop_trigger_service = str(self.get_parameter("teleop_trigger_service").value)

        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_max_range = float(self.get_parameter("gripper_max_range").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.hand_name = str(self.get_parameter("hand_name").value)
        
        self.handle_pose_roll = float(self.get_parameter("handle_pose_roll").value)
        self.handle_pose_pitch = float(self.get_parameter("handle_pose_pitch").value)
        self.handle_pose_yaw = float(self.get_parameter("handle_pose_yaw").value)

        self.pub_delta_pose = self.create_publisher(PoseStamped, delta_pose_topic, 10)
        #self.pub_move_j = self.create_publisher(JointState, control_joint_topic, 10)
        
        # Cached pose matrices
        self.handle_matrix = None
        self.tcp_matrix = None
        self.gripper_position = []

        self.teleop_active = False
        self.tcp_zero_initialized = False
        self._start_pose_inv = None
        self.zero_matrix = np.eye(4, dtype=float)
        self._baseline_matrix = None
        self._handle_pose_rot_offset = Rotation.from_euler(
            "xyz",
            [self.handle_pose_roll, self.handle_pose_pitch, self.handle_pose_yaw],
        ).as_matrix()

        self.create_subscription(PoseStamped, handle_pose_topic, self.handle_pose_callback, 1)
        self.create_subscription(PoseStamped, feedback_tcp_pose_topic, self.tcp_pose_callback, 1)
        self.create_subscription(JointState, gripper_joint_state_topic, self.gripper_joint_state_callback, 1)
        
        self.status_srv = self.create_service(Trigger, teleop_trigger_service, self.teleop_trigger_callback)

        self.control_timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.get_logger().info(
            f"pub_delta_pose ready ({self.hand_name}). "
            f"handle_topic={handle_pose_topic}, feedback_topic={feedback_tcp_pose_topic}, "
            f"delta_topic={delta_pose_topic}, control_topic={control_joint_topic}, "
            f"gripper_joint_state_topic={gripper_joint_state_topic}"
        )

    def _rebuild_baseline_matrix(self):
        if self._start_pose_inv is None:
            self._baseline_matrix = None
            return
        self._baseline_matrix = self.zero_matrix @ self._start_pose_inv

    def tcp_pose_callback(self, msg: PoseStamped):
        self.tcp_matrix = pose_msg_to_mat(msg)
        if not self.tcp_zero_initialized:
            self.zero_matrix = self.tcp_matrix.copy()
            self.tcp_zero_initialized = True
            self._rebuild_baseline_matrix()

    def handle_pose_callback(self, msg: PoseStamped):
        self.handle_matrix = pose_msg_to_mat(msg)
        self.handle_matrix[:3, :3] = self.handle_matrix[:3, :3] @ self._handle_pose_rot_offset

    def gripper_joint_state_callback(self, msg: JointState):
        self.gripper_position = list(msg.position)

    def teleop_trigger_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.teleop_active:
            self.teleop_active = False
            self._start_pose_inv = None
            if self.tcp_matrix is not None:
                self.zero_matrix = self.tcp_matrix.copy()
            self._baseline_matrix = None
            self.get_logger().info(f"[{self.hand_name}] 停止遥操作")
        else:
            if self.handle_matrix is None or self.tcp_matrix is None:
                response.success = False
                response.message = "waiting for handle/tcp pose feedback"
                return response
            self.teleop_active = True
            self._start_pose_inv = np.linalg.inv(self.handle_matrix)
            self._rebuild_baseline_matrix()
            self.get_logger().info(f"[{self.hand_name}] 开始遥操作")

        response.success = True
        response.message = "ok"
        return response

    def control_loop(self):
        if not self.teleop_active:
            return

        if self.handle_matrix is None or self._baseline_matrix is None:
            return
        
        # gripper coltrol
        # gripper_msg = JointState()
        # gripper_msg.header = Header()
        # gripper_msg.header.stamp = self.get_clock().now().to_msg()
        # gripper_msg.name = [self.gripper_joint_name]
        # gripper_msg.position = list(self.gripper_position)
        #self.pub_move_j.publish(gripper_msg)

        result_matrix = self._baseline_matrix @ self.handle_matrix
        xyz, quat = mat2xyzquat(result_matrix)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = float(xyz[0])
        pose_msg.pose.position.y = float(xyz[1])
        pose_msg.pose.position.z = float(xyz[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.pub_delta_pose.publish(pose_msg)

    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = None
    try:
        teleop_node = RosOperator()
        executor = MultiThreadedExecutor()
        executor.add_node(teleop_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if teleop_node is not None:
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
