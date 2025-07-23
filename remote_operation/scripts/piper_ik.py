#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import os
import sys
import math
import numpy as np
import pinocchio as pin

# 获取当前脚本所在的目录路径，并将其存储在变量 ROOT_DIR 中
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR,'dist_pyarmor'))
from wrapper import ik_fun,get_pose

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
    def __init__(self):  
        rospy.init_node('piper_IK', anonymous=True)  #  /piper_FK/urdf_end_pose  /piper_IK/ctrl_end_pose
        self.index_name = rospy.get_param('~index_name', default="")
        self.gripper_position = rospy.get_param('~gripper_xyzrpy', default=[0.19, 0.0, 0.2, 0.0, 0.0, 0.0])
        self.target_joint_state = rospy.get_param('~target_joint_state', default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.arm_joint_state_publisher = None
        self.arm_end_pose_publisher = None
        self.arm_end_pose_orient_publisher = None
        self.arm_receive_end_pose_publisher = None
        self.takeover = False
        self.flag = False
        self.gripper_value = 0
        self.base_pose = self.gripper_position   # 末端夹爪到基坐标系的变换
        
        # 存储当前关节位置
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # 标记是否已获取到当前关节位置
        self.joint_positions_received = False
        
        self.init_ros()

    def calc_pose_incre(self,base_pose, pose_data):
        begin_matrix = create_transformation_matrix(base_pose[0], base_pose[1], base_pose[2],
                                                    base_pose[3], base_pose[4], base_pose[5])
        zero_matrix = create_transformation_matrix(self.gripper_position[0],self.gripper_position[1],self.gripper_position[2],
                                                   self.gripper_position[3],self.gripper_position[4],self.gripper_position[5],)
        end_matrix = create_transformation_matrix(pose_data[0], pose_data[1], pose_data[2],
                                                pose_data[3], pose_data[4], pose_data[5])
        result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
        xyzrpy = matrix_to_xyzrpy(result_matrix)
        return xyzrpy

    def arm_end_pose_callback(self, msg):
        # print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        RR = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
        RR_ = self.calc_pose_incre(self.base_pose,RR)
        
        q = quaternion_from_euler(RR_[3],RR_[4],RR_[5])
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([RR_[0],RR_[1],RR_[2]]),
        )
        sol_q, tau_ff, get_result = ik_fun(target.homogeneous, gripper=0, motorstate=None, motorV=None)
        # print("result:", sol_q, tau_ff)pyarmor gen --output=dist_pyarmor example.py wrapper.py
        if self.flag:  # get_result:
            joint_position = []
            joint_position.append(sol_q[0])
            joint_position.append(sol_q[1])
            joint_position.append(sol_q[2])
            joint_position.append(sol_q[3])
            joint_position.append(sol_q[4])
            joint_position.append(sol_q[5])
            
            #夹爪值
            joint_position.append(self.gripper_value)
            
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = [f'joint{i+1}' for i in range(8)]
            joint_state_msg.position = joint_position
            # joint_state_msg.position.append(msg.pose.orientation.w / 2.0)
            # joint_state_msg.position.append(-msg.pose.orientation.w / 2.0)
            joint_state_msg.position.append(msg.pose.orientation.w)
            self.arm_joint_state_publisher.publish(joint_state_msg)

            xyzrpy = get_pose(joint_position[:6])
            end_pose_msg = PoseStamped()
            end_pose_msg.header = Header()
            end_pose_msg.header.stamp = rospy.Time.now()
            end_pose_msg.header.frame_id = "map"
            end_pose_msg.pose.position.x = xyzrpy[0]
            end_pose_msg.pose.position.y = xyzrpy[1]
            end_pose_msg.pose.position.z = xyzrpy[2]
            end_pose_msg.pose.orientation.x = xyzrpy[3]
            end_pose_msg.pose.orientation.y = xyzrpy[4]
            end_pose_msg.pose.orientation.z = xyzrpy[5]
            # end_pose_msg.pose.orientation.w = joint_position[-2] - joint_position[-1]
            end_pose_msg.pose.orientation.w = msg.pose.orientation.w
            self.arm_end_pose_publisher.publish(end_pose_msg)
            x, y, z, w = quaternion_from_euler(end_pose_msg.pose.orientation.x, end_pose_msg.pose.orientation.y, end_pose_msg.pose.orientation.z)
            end_pose_msg.pose.orientation.x = x
            end_pose_msg.pose.orientation.y = y
            end_pose_msg.pose.orientation.z = z
            end_pose_msg.pose.orientation.w = w
            self.arm_end_pose_orient_publisher.publish(end_pose_msg)
            end_pose_msg.pose.position.x = msg.pose.position.x
            end_pose_msg.pose.position.y = msg.pose.position.y
            end_pose_msg.pose.position.z = msg.pose.position.z
            end_pose_msg.pose.orientation.x = msg.pose.orientation.x
            end_pose_msg.pose.orientation.y = msg.pose.orientation.y
            end_pose_msg.pose.orientation.z = msg.pose.orientation.z
            x, y, z, w = quaternion_from_euler(end_pose_msg.pose.orientation.x, end_pose_msg.pose.orientation.y, end_pose_msg.pose.orientation.z)
            end_pose_msg.pose.orientation.x = x
            end_pose_msg.pose.orientation.y = y
            end_pose_msg.pose.orientation.z = z
            end_pose_msg.pose.orientation.w = w
            self.arm_receive_end_pose_publisher.publish(end_pose_msg)
    
    def gripper_callback(self, msg):
        self.gripper_value = msg.position[0]
        # print("gripper_value:", self.gripper_value)
        
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
        # 目标关节位置
        target_joint_state = self.target_joint_state
        
        # 获取当前关节位置
        # 如果已经接收到关节位置数据，使用实际的当前位置
        # 否则会一步调整到位
        if self.joint_positions_received:
            current_positions = self.current_joint_positions
            rospy.loginfo(f"使用实际的当前关节位置: {current_positions}")
            
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
            # print("send joint control piper command for 2 seconds")
            # 使用默认的非零位置作为起始点
            # current_positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # rospy.loginfo("未接收到当前关节位置，使用默认初始位置")
        
    def handle_trigger(self,req):
        rospy.loginfo("Service /trigger has been called")
        
        # 在这里执行你想要触发的操作
        self.takeover = not self.takeover
        
        if self.takeover is True:
            self.base_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            self.flag = True
            print("开始遥操作")
        else:
            self.flag = False
            # 检测到复位，机械臂回到初始点位并且记录 右 坐标原点
            self.init_pose()
            self.base_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            print("机械臂已复位")
        
        success = True  # 假设操作总是成功
        
        if success:
            rospy.loginfo("Triggered successfully")
            return TriggerResponse(
                success=True,
                message="Triggered successfully"
            )
        else:
            rospy.loginfo("Failed to trigger")
            return TriggerResponse(
                success=False,
                message="Failed to trigger"
            )
    def init_ros(self):
        # rospy.init_node(f'piper_IK{self.args.index_name}', anonymous=True)  #  /piper_FK/urdf_end_pose  /piper_IK/ctrl_end_pose
        # rospy.Subscriber(f'/piper_IK{self.args.index_name}/ctrl_end_pose', PoseStamped, self.arm_end_pose_callback, queue_size=1)
        self.arm_joint_state_publisher = rospy.Publisher(f'/joint_states_gripper{self.index_name}', JointState, queue_size=10)
        self.arm_end_pose_publisher = rospy.Publisher(f'/piper_IK{self.index_name}/urdf_end_pose', PoseStamped, queue_size=10)
        self.arm_end_pose_orient_publisher = rospy.Publisher(f'/piper_IK{self.index_name}/urdf_end_pose_orient', PoseStamped, queue_size=10)
        self.arm_receive_end_pose_publisher = rospy.Publisher(f'/piper_IK{self.index_name}/receive_end_pose_orient', PoseStamped, queue_size=10)
        
        #-----------
        rospy.Subscriber(f'/pika_pose{self.index_name}', PoseStamped, self.arm_end_pose_callback, queue_size=1)
        rospy.Service(f'/teleop_trigger{self.index_name}',Trigger, self.handle_trigger)        
        rospy.Subscriber(f'/gripper{self.index_name}/joint_state', JointState, self.gripper_callback, queue_size=1)
        # 订阅joint_states_single话题获取当前关节位置
        rospy.Subscriber(f'joint_states_single{self.index_name}', JointState, self.joint_states_callback, queue_size=1)
        import time 
        time.sleep(0.5)
        self.init_pose()

def main():
    ros_operator = RosOperator()
    rospy.spin()


if __name__ == "__main__":
    main()
