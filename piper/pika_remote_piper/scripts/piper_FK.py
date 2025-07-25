#!/usr/bin/env python3
import casadi
import meshcat.geometry as mg
import math
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
from transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
import os
import sys
import cv2
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import threading

# 获取当前脚本所在的目录路径，并将其存储在变量 ROOT_DIR 中
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR,'dist_pyarmor_FIK'))
from wrapper import Arm_FK


os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


class RosOperator:
    def __init__(self, args):
        self.args = args
        self.lift_subscriber = None
        self.arm_joint_state_subscriber = None
        self.arm_end_pose_publisher = None
        self.arm_end_pose_orient_publisher = None
        self.arm_msg = None
        self.lift_msg = None
        self.calc_thread = None
        self.init_ros()
        self.arm_fk = Arm_FK(args)

    def lift_callback(self, msg):
        self.lift_msg = msg

    def arm_joint_state_callback(self, msg):
        self.arm_msg = msg

    def start(self):
        self.calc_thread = threading.Thread(target=self.calc)
        self.calc_thread.start()

    def calc(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            if (self.args.lift and self.lift_msg is None) or self.arm_msg is None:
                rate.sleep()
                continue
            xyzrpy = self.arm_fk.get_pose((self.lift_msg.position if self.args.lift else ()) + self.arm_msg.position[:6])
            end_pose_msg = PoseStamped()
            end_pose_msg.header = Header()
            end_pose_msg.header.stamp = rospy.Time.now()
            end_pose_msg.header.frame_id = "base_link"
            end_pose_msg.pose.position.x = xyzrpy[0]
            end_pose_msg.pose.position.y = xyzrpy[1]
            end_pose_msg.pose.position.z = xyzrpy[2]
            end_pose_msg.pose.orientation.x = xyzrpy[3]
            end_pose_msg.pose.orientation.y = xyzrpy[4]
            end_pose_msg.pose.orientation.z = xyzrpy[5]
            end_pose_msg.pose.orientation.w = self.arm_msg.position[6]
            self.arm_end_pose_publisher.publish(end_pose_msg)
            x, y, z, w = quaternion_from_euler(end_pose_msg.pose.orientation.x, end_pose_msg.pose.orientation.y, end_pose_msg.pose.orientation.z)
            end_pose_msg.pose.orientation.x = x
            end_pose_msg.pose.orientation.y = y
            end_pose_msg.pose.orientation.z = z
            end_pose_msg.pose.orientation.w = w
            self.arm_end_pose_orient_publisher.publish(end_pose_msg)
            # print("end_pose:", xyzrpy)
            rate.sleep()

    def init_ros(self):
        rospy.init_node(f'piper_FK{self.args.index_name}', anonymous=True)
        self.args.index_name = rospy.get_param('~index_name', default="")
        self.args.gripper_xyzrpy = rospy.get_param('~gripper_xyzrpy', default=[0.19, 0.0, 0.0, 0.0, 0.0, 0.0])
        if self.args.lift:
            self.lift_subscriber = rospy.Subscriber(f'/joint_states_single_lift', JointState, self.lift_callback, queue_size=1)
        self.arm_joint_state_subscriber = rospy.Subscriber(f'/joint_states_single{self.args.index_name}', JointState, self.arm_joint_state_callback, queue_size=1)
        self.arm_end_pose_publisher = rospy.Publisher(f'/piper_FK{self.args.index_name}/urdf_end_pose', PoseStamped, queue_size=1)
        self.arm_end_pose_orient_publisher = rospy.Publisher(f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', PoseStamped, queue_size=1)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lift', action='store', type=bool, help='lift',
                        default=False, required=False)
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    parser.add_argument('--gripper_xyzrpy', action='store', nargs='+', type=float, help='gripper_xyzrpy',
                        default=[0.19, 0, 0, 0, 0, 0], required=False)
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    return args

#     def init_ros(self):
#         rospy.init_node(f'piper_FK{self.args.index_name}', anonymous=True)
#         if self.args.lift:
#             self.lift_subscriber = rospy.Subscriber(f'/joint_states_single_lift', JointState, self.lift_callback, queue_size=1)
#         self.arm_joint_state_subscriber = rospy.Subscriber(f'/joint_states_single{self.args.index_name}', JointState, self.arm_joint_state_callback, queue_size=1)
#         self.arm_end_pose_publisher = rospy.Publisher(f'/piper_FK{self.args.index_name}/urdf_end_pose', PoseStamped, queue_size=1)
#         self.arm_end_pose_orient_publisher = rospy.Publisher(f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', PoseStamped, queue_size=1)
#
#
# def get_arguments():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--lift', action='store', type=bool, help='lift',
#                         default=False, required=False)
#     parser.add_argument('--index_name', action='store', type=str, help='index_name',
#                         default="", required=False)
#     parser.add_argument('--gripper_xyzrpy', action='store', nargs='+', type=float, help='gripper_xyzrpy',
#                         default=[0, 0, 0, 0, 0, 0], required=False)
#     args = parser.parse_args()
#     return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    ros_operator.start()
    rospy.spin()


if __name__ == "__main__":
    main()


# def update_data(data):
#     return
#
#
# if __name__ == "__main__":
#     rospy.init_node('piper_FK', anonymous=True)
#     args = get_arguments()
#     arm_joint_state_publisher = rospy.Publisher('/piper_FK/cv_joint_state', JointState, queue_size=1)
#     arm_end_pose_publisher = rospy.Publisher('/piper_FK/urdf_end_pose', PoseStamped, queue_size=1)
#
#     cv2.namedWindow('joint_controller', cv2.WINDOW_NORMAL)
#     cv2.createTrackbar('joint1', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('joint2', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('joint3', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('joint4', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('joint5', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('joint6', 'joint_controller', 0, 100, update_data)
#     cv2.createTrackbar('gripper', 'joint_controller', 0, 100, update_data)
#     cv2.setTrackbarPos('joint1', 'joint_controller', 50)
#     cv2.setTrackbarPos('joint2', 'joint_controller', 0)
#     cv2.setTrackbarPos('joint3', 'joint_controller', 100)
#     cv2.setTrackbarPos('joint4', 'joint_controller', 50)
#     cv2.setTrackbarPos('joint5', 'joint_controller', 50)
#     cv2.setTrackbarPos('joint6', 'joint_controller', 50)
#     cv2.setTrackbarPos('gripper', 'joint_controller', 0)
#     cv2.imshow('joint_controller', np.ones((100, 100, 3), dtype=np.uint8) * 255)
#     ctrl_rate = rospy.Rate(100)
#     arm_fk = Arm_FK(args)
#     while not rospy.is_shutdown():
#         joint_position = []
#         joint_position.append(-2.618 + cv2.getTrackbarPos('joint1', 'joint_controller') * (2.618 - -2.618) / 100)
#         joint_position.append(0 + cv2.getTrackbarPos('joint2', 'joint_controller') * (3.14 - 0) / 100)
#         joint_position.append(-2.967 + cv2.getTrackbarPos('joint3', 'joint_controller') * (0 - -2.967) / 100)
#         joint_position.append(-1.832 + cv2.getTrackbarPos('joint4', 'joint_controller') * (1.832 - -1.832) / 100)
#         joint_position.append(-1.22 + cv2.getTrackbarPos('joint5', 'joint_controller') * (1.22 - -1.22) / 100)
#         joint_position.append(-3.14 + cv2.getTrackbarPos('joint6', 'joint_controller') * (3.14 - -3.14) / 100)
#         joint_position.append(0 + cv2.getTrackbarPos('gripper', 'joint_controller') * (0.08 - 0) / 100 / 2)
#         joint_position.append(0 + cv2.getTrackbarPos('gripper', 'joint_controller') * (0 - 0.08) / 100 / 2)
#
#         joint_state_msg = JointState()
#         joint_state_msg.header = Header()
#         joint_state_msg.header.stamp = rospy.Time.now()
#         joint_state_msg.name = [f'joint{i + 1}' for i in range(8)]
#         joint_state_msg.position = joint_position
#         arm_joint_state_publisher.publish(joint_state_msg)
#         print("joint_position:", joint_position)
#
#         xyzrpy = arm_fk.get_pose(joint_position[:6])
#         end_pose_msg = PoseStamped()
#         end_pose_msg.header = Header()
#         end_pose_msg.header.stamp = rospy.Time.now()
#         end_pose_msg.pose.position.x = xyzrpy[0]
#         end_pose_msg.pose.position.y = xyzrpy[1]
#         end_pose_msg.pose.position.z = xyzrpy[2]
#         end_pose_msg.pose.orientation.x = xyzrpy[3]
#         end_pose_msg.pose.orientation.y = xyzrpy[4]
#         end_pose_msg.pose.orientation.z = xyzrpy[5]
#         end_pose_msg.pose.orientation.w = joint_position[-2] - joint_position[-1]
#         arm_end_pose_publisher.publish(end_pose_msg)
#         print("end_pose:", xyzrpy)
#
#         ctrl_rate.sleep()
#         if cv2.waitKey(1) == ord('q'):
#             break
