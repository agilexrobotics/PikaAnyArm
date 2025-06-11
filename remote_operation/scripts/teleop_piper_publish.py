#!/usr/bin/env python3
import math
import numpy as np
from transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
import os
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import argparse
from nav_msgs.msg import Odometry
import threading
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from data_msgs.msg import TeleopStatus


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

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
        self.localization_pose_subscriber = None
        self.arm_end_pose_subscriber = None

        self.arm_end_pose_ctrl_publisher = None

        self.localization_pose_matrix = None
        self.arm_end_pose_matrix = None

        self.refresh_localization_pose = True
        self.refresh_arm_end_pose = True

        self.status_srv = None
        self.status = False
        self.init_ros()

    def localization_pose_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        matrix = create_transformation_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
        if self.refresh_localization_pose:
            self.refresh_localization_pose = False
            self.localization_pose_matrix = matrix
        if self.arm_end_pose_matrix is not None and self.status:
            pose_xyzrpy = matrix_to_xyzrpy(np.dot(self.arm_end_pose_matrix, np.dot(np.linalg.inv(self.localization_pose_matrix), matrix)))
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = pose_xyzrpy[0]
            pose_msg.pose.position.y = pose_xyzrpy[1]
            pose_msg.pose.position.z = pose_xyzrpy[2]
            q = quaternion_from_euler(pose_xyzrpy[3], pose_xyzrpy[4], pose_xyzrpy[5])
            pose_msg.pose.orientation.x = pose_xyzrpy[3]  # q[0]
            pose_msg.pose.orientation.y = pose_xyzrpy[4]  # q[1]
            pose_msg.pose.orientation.z = pose_xyzrpy[5]  # q[2]
            pose_msg.pose.orientation.w = 0  # q[3]
            self.arm_end_pose_ctrl_publisher.publish(pose_msg)
            status_msg = TeleopStatus()
            status_msg.teleop = True
            status_msg.wait = False
            self.teleop_status_publisher.publish(status_msg)

    def arm_end_pose_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        matrix = create_transformation_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
        if self.refresh_arm_end_pose:
            self.refresh_arm_end_pose = False
            self.arm_end_pose_matrix = matrix

    def teleop_piper_status_callback(self, req):
        if req.data:
            self.refresh_localization_pose = True
            self.refresh_arm_end_pose = True
            rate = rospy.Rate(10)
            print_flag = True
            while not rospy.is_shutdown():
                if not self.refresh_localization_pose and not self.refresh_arm_end_pose:
                    print("start")
                    break
                status_msg = TeleopStatus()
                status_msg.teleop = False
                status_msg.wait = True
                self.teleop_status_publisher.publish(status_msg)
                if print_flag:
                    print_flag = False
                    print("wait msg")
                rate.sleep()
            self.status = True
        else:
            self.status = False
            print("close")
        return SetBoolResponse()

    def init_ros(self):
        rospy.init_node(f'teleop_piper_publisher{self.args.index_name}', anonymous=True)
        self.args.index_name = rospy.get_param('~index_name', default="")
        self.localization_pose_subscriber = rospy.Subscriber(f'/pika_pose{self.args.index_name}', PoseStamped, self.localization_pose_callback, queue_size=1)
        self.arm_end_pose_subscriber = rospy.Subscriber(f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', PoseStamped, self.arm_end_pose_callback, queue_size=1)
        self.arm_end_pose_ctrl_publisher = rospy.Publisher(f'/piper_IK{self.args.index_name}/ctrl_end_pose', PoseStamped, queue_size=1)
        self.teleop_status_publisher = rospy.Publisher(f'/teleop_status{self.args.index_name}', TeleopStatus, queue_size=1)
        self.status_srv = rospy.Service(f'/teleop_piper_status{self.args.index_name}', SetBool, self.teleop_piper_status_callback)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    rospy.spin()


if __name__ == "__main__":
    main()
