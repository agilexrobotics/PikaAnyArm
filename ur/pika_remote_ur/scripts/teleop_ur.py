#!/usr/bin/env python3
import rospy
import time
import numpy as np
from tools import MATHTOOLS

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion

# UR机械臂控制
from ur_control import URCONTROL

tools = MATHTOOLS()

class Arm_IK:
    def __init__(self):
        rospy.init_node('remote_xarm_node', anonymous=True)
        self.index_name = rospy.get_param('~index_name', default="")
        self.robot_ip = rospy.get_param('~robot_ip', default="192.168.1.5")
        self.ur_control = URCONTROL(self.robot_ip)
        
        self.initial_pose_rotvec = self.ur_control.get_tcp_pose()

        self.pika_to_arm = rospy.get_param('~pika_to_arm', default=[0,0,0,   0, 1.570796, 0])
               
        temp_rotvec = [self.initial_pose_rotvec[3],self.initial_pose_rotvec[4],self.initial_pose_rotvec[5]]
        
        # 将旋转向量转换成欧拉角
        roll, pitch, yaw = tools.rotvec_to_rpy(temp_rotvec)
        
        # self.initial_pose_rpy = self.initial_pose_rotvec
        self.initial_pose_rpy = self.initial_pose_rotvec[:]

        self.initial_pose_rpy[3] = roll
        self.initial_pose_rpy[4] = pitch
        self.initial_pose_rpy[5] = yaw
        
        self.init_ros()
        
        # 遥操控制频率
        self.rate = rospy.Rate(50)
        
        self.takeover = False
              
        self.running = True
        
        self.flag = False
        
        # 单机械臂的初始位置
        self.base_pose = self.initial_pose_rpy #想要的目标姿态数据
        
        time.sleep(2)
        
        # 这里是为了防止程序启动时，pika_pose 数据为空的时候，程序执行失败
        self.x, self.y, self.z = self.initial_pose_rpy[0],self.initial_pose_rpy[1],self.initial_pose_rpy[2]
        self.roll, self.pitch, self.yaw = self.initial_pose_rpy[3],self.initial_pose_rpy[4],self.initial_pose_rpy[5]

    def handle_trigger(self,req):
        rospy.loginfo("Service /trigger has been called")
        
        # 在这里执行你想要触发的操作
        self.takeover = not self.takeover
        print("反转",self.takeover)
        
        if self.takeover is True:
            self.base_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            self.flag = True
            print("开始遥操作")
            
        else:
            self.flag = False
            
            #-------------------------------------------------方案一：遥操结束机械臂停止在当前位姿，下次遥操开始继续从当前位姿开始---------------------------------------------------

            self.initial_pose_rotvec = self.ur_control.get_tcp_pose()
            
            temp_rotvec = [self.initial_pose_rotvec[3],self.initial_pose_rotvec[4],self.initial_pose_rotvec[5]]
        
            # 将旋转向量转换成欧拉角
            roll, pitch, yaw = tools.rotvec_to_rpy(temp_rotvec)
            
            # self.initial_pose_rpy = self.initial_pose_rotvec
            self.initial_pose_rpy = self.initial_pose_rotvec[:]

            self.initial_pose_rpy[3] = roll
            self.initial_pose_rpy[4] = pitch
            self.initial_pose_rpy[5] = yaw
            
            self.base_pose = self.initial_pose_rpy #想要的目标姿态数据
            
            print("停止遥操")
            
            #-------------------------------------------------方案二：遥操结束机械臂回到初始位姿，下次遥操开始从初始位姿开始---------------------------------------------------
            
            # # 获取当前机械臂的位姿
            # current_pose = self.ur_control.get_tcp_pose()

            # # 定义插值步数
            # num_steps = 100  # 可以根据需要调整步数，步数越多，过渡越平滑

            # for i in range(1, num_steps + 1):
            #     # 计算当前插值点位姿
            #     # 这里假设 initial_pose_rotvec 和 current_pose 都是 [x, y, z, Rx, Ry, Rz] 格式
            #     interpolated_pose = [
            #         current_pose[j] + (self.initial_pose_rotvec[j] - current_pose[j]) * i / num_steps
            #         for j in range(6)
            #     ]
            #     self.ur_control.sevol_l(interpolated_pose)
            #     time.sleep(0.01)  # 每次插值之间稍作延时，控制速度

            # # 确保最终到达初始位置
            # self.ur_control.sevol_l(self.initial_pose_rotvec)


            # self.base_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            # print("停止遥操")
            
            
            
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

    # 增量式控制
    def calc_pose_incre(self,base_pose, pose_data):
        begin_matrix = tools.xyzrpy2Mat(base_pose[0], base_pose[1], base_pose[2],
                                                    base_pose[3], base_pose[4], base_pose[5])
        zero_matrix = tools.xyzrpy2Mat(self.initial_pose_rpy[0],self.initial_pose_rpy[1],self.initial_pose_rpy[2],
                                            self.initial_pose_rpy[3],self.initial_pose_rpy[4],self.initial_pose_rpy[5])
        end_matrix = tools.xyzrpy2Mat(pose_data[0], pose_data[1], pose_data[2],
                                                pose_data[3], pose_data[4], pose_data[5])
        result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
        xyzrpy = tools.mat2xyzrpy(result_matrix)
        return xyzrpy
    
    # 订阅pika_pose回调函数
    def pose_callback(self,data):
        
        x = data.pose.position.x   
        y = data.pose.position.y 
        z = data.pose.position.z 
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
                
        self.x,self.y,self.z,   self.roll, self.pitch, self.yaw = self.adjustment(x,y,z,roll,pitch,yaw)
                
    # 调整矩阵函数
    def adjustment(self,x,y,z,Rx,Ry,Rz):
        transform = tools.xyzrpy2Mat(x,y,z,   Rx, Ry, Rz)

        r_adj = tools.xyzrpy2Mat(self.pika_to_arm[0],self.pika_to_arm[1],self.pika_to_arm[2],
                                      self.pika_to_arm[3],self.pika_to_arm[4],self.pika_to_arm[5],)   # 调整坐标轴方向  pika--->机械臂末端

        transform = np.dot(transform, r_adj)
        
        x_,y_,z_,Rx_,Ry_,Rz_ = tools.mat2xyzrpy(transform)
        
        return x_,y_,z_,Rx_,Ry_,Rz_
            
    # ROS Subscriber and Service
    def init_ros(self):
        rospy.Subscriber(f'/pika_pose{self.index_name}', PoseStamped, self.pose_callback, queue_size=1)
                
        rospy.Service(f'/teleop_trigger{self.index_name}',Trigger, self.handle_trigger)

    def start(self):

        # 主线程继续执行其他任务
        while self.running and not rospy.is_shutdown():
            self.rate.sleep() # 50HZ
            current_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
            increment_pose = self.calc_pose_incre(self.base_pose,current_pose)
            
            finally_pose  = tools.rpy_to_rotvec(increment_pose[3], increment_pose[4], increment_pose[5])
            
            increment_pose[3:6] = finally_pose
            
            #下发pose至机械臂
            if self.flag:
                self.ur_control.sevol_l(increment_pose)
                
    def stop(self):
        self.running = False   

if __name__ == "__main__":
    
    system = Arm_IK()
    try:
        system.start()
    except KeyboardInterrupt:
        system.stop()
        print("程序已退出")
    rospy.spin()
    
    
    