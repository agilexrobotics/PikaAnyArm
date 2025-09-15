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
from sensor_msgs.msg import JointState


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

        self.thread = None
        self.stop_thread = False

        self.status_srv = None
        self.status = False
        
        # 存储当前关节位置
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # 标记是否已获取到当前关节位置
        self.joint_positions_received = False
        
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
            status_msg.quit = False
            status_msg.fail = False
            self.teleop_status_publisher.publish(status_msg)

    def arm_end_pose_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        matrix = create_transformation_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
        if self.refresh_arm_end_pose:
            self.refresh_arm_end_pose = False
            self.arm_end_pose_matrix = matrix

    def status_changing(self):
        self.refresh_localization_pose = True
        self.refresh_arm_end_pose = True
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.stop_thread:
                self.status = False
                self.refresh_localization_pose = False
                self.refresh_arm_end_pose = False
                break
            if not self.refresh_localization_pose and not self.refresh_arm_end_pose:
                print("start")
                self.status = True
                break
            else:
                status_msg = TeleopStatus()
                status_msg.quit = False
                status_msg.fail = True
                print("wait")
                self.teleop_status_publisher.publish(status_msg)
            rate.sleep()

    def teleop_trigger_callback(self, req):
        if self.status:
            self.status = False
            status_msg = TeleopStatus()
            status_msg.quit = True
            status_msg.fail = False
            self.teleop_status_publisher.publish(status_msg)
            print("close")
            self.init_pose()
        else:
            if self.thread is None or not self.thread.is_alive():
                self.stop_thread = False
                self.thread = threading.Thread(target=self.status_changing)
                self.thread.start()
            else:
                self.stop_thread = True
                self.thread.join()
                self.status = False
                status_msg = TeleopStatus()
                status_msg.quit = True
                status_msg.fail = False
                self.teleop_status_publisher.publish(status_msg)
                print("close")
                self.init_pose()
        return TriggerResponse()

    def init_ros(self):
        rospy.init_node(f'teleop_piper_publisher{self.args.index_name}', anonymous=True)
        self.args.index_name = rospy.get_param('~index_name', default="")
        self.localization_pose_subscriber = rospy.Subscriber(f'/pika_pose{self.args.index_name}', PoseStamped, self.localization_pose_callback, queue_size=1)
        self.arm_end_pose_subscriber = rospy.Subscriber(f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', PoseStamped, self.arm_end_pose_callback, queue_size=1)
        self.arm_end_pose_ctrl_publisher = rospy.Publisher(f'/piper_IK{self.args.index_name}/ctrl_end_pose', PoseStamped, queue_size=1)
        self.teleop_status_publisher = rospy.Publisher(f'/teleop_status{self.args.index_name}', TeleopStatus, queue_size=1)
        self.status_srv = rospy.Service(f'/teleop_trigger{self.args.index_name}', Trigger, self.teleop_trigger_callback)
        
        self.arm_joint_state_publisher = rospy.Publisher(f'/joint_states{self.args.index_name}', JointState, queue_size=10)
        self.args.return_zero_position = rospy.get_param('~return_zero_position', default="False")
        # 订阅joint_states_single话题获取当前关节位置
        rospy.Subscriber(f'/joint_states_gripper{self.args.index_name}', JointState, self.joint_states_callback, queue_size=1)
        import time 
        time.sleep(0.5)
        self.init_pose()
        
    def joint_states_callback(self, msg):
        """
        处理从joint_states_single话题接收到的关节状态数据
        """
        # 确保消息中包含关节位置数据
        if len(msg.position) >= 7:
            # 更新当前关节位置
            self.current_joint_positions = list(msg.position[:7])
            # 标记已接收到关节位置数据
            self.joint_positions_received = True
            # rospy.logdebug(f"接收到当前关节位置: {self.current_joint_positions}")
        
    # 使用线性插值实现平滑过渡到初始位置
    def init_pose(self):
        if self.args.return_zero_position == "True":
            # 目标关节位置
            target_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 获取当前关节位置
            # 如果已经接收到关节位置数据，使用实际的当前位置
            # 否则会一步调整到位
            if self.joint_positions_received:
                current_positions = self.current_joint_positions
                
                # 设置过渡时间和控制频率
                duration = 0.5  # 过渡持续时间(秒)
                rate = 50  # 控制频率(Hz)
                
                # 计算总步数
                steps = int(duration * rate)
                
                # 计算每一步的增量
                increments = [(target - current) / steps for current, target in zip(current_positions, target_joint_state)]
                
                # 创建ROS的Rate对象控制循环频率
                rate_obj = rospy.Rate(rate)
                
                # 记录开始时间（用于日志）
                start_time = rospy.Time.now()
                
                # 逐步移动到目标位置
                for step in range(steps + 1):
                    # 计算当前步骤的位置
                    interpolated_positions = [current + increment * step for current, increment in zip(current_positions, increments)]
                    
                    # 发布关节状态消息
                    joint_states_msgs = JointState()
                    joint_states_msgs.header = Header()
                    joint_states_msgs.header.stamp = rospy.Time.now()
                    joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                    joint_states_msgs.position = interpolated_positions
                    
                    # 发布消息
                    self.arm_joint_state_publisher.publish(joint_states_msgs)
                    
                    # 按照指定频率控制循环
                    rate_obj.sleep()
                
                # 确保最后一帧是精确的目标位置
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = rospy.Time.now()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = target_joint_state
                self.arm_joint_state_publisher.publish(joint_states_msgs)
                
                # 计算实际用时
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                # print(f"平滑移动到初始位置完成，用时: {elapsed_time:.2f}秒")
                
            else:
                start_time = rospy.Time.now()  # 获取当前时间
                while (rospy.Time.now() - start_time).to_sec() < 0.5:  # 持续发送0.5秒
                    joint_states_msgs = JointState()
                    joint_states_msgs.header = Header()
                    joint_states_msgs.header.stamp = rospy.Time.now()
                    joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                    joint_states_msgs.position = target_joint_state
                    self.arm_joint_state_publisher.publish(joint_states_msgs)
        else:
            return
                
def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    parser.add_argument('--return_zero_position', action='store', type=str, help='return_zero_position',
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
    
