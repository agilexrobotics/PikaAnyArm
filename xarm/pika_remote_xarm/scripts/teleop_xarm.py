#!/usr/bin/env python3
import rospy
import time
import numpy as np
from tools import MATHTOOLS
from xarm_control import XARM
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion

xarm_control = XARM()

class Arm_IK:
    def __init__(self):
        rospy.init_node('remote_xarm_node', anonymous=True)
        self.index_name = rospy.get_param('~index_name', default="")
        self.eff_position = rospy.get_param('~eff_position', default='[223.778564, 0.357308, 285.864502, -0.390661, 1.521898, -0.379501]')
        self.pika_to_arm = rospy.get_param('~pika_to_arm', default='[0,0,0,   0, 1.570796, 0]')
        
        # 单机械臂的初始位置
        self.base_pose = self.eff_position #想要的目标姿态数据
        
        # 将机械臂全部点击使能且设置为伺服模式
        xarm_control.set_motion_ctrl(8,1)
        time.sleep(3)
        
        # 设置机械臂模式为7
        xarm_control.set_mode(7)
        time.sleep(1)
        # 设置机械臂状态为0
        xarm_control.set_state(0)
        
        self.init_ros()
        
        self.tools = MATHTOOLS()
        
        # 遥操控制频率
        self.rate = rospy.Rate(50)
        
        self.takeover = False
              
        self.running = True
        
        self.flag = False
        
        #机械臂回到初始点位
        xarm_control.moveL(self.eff_position,mode=0)
        
        # 这里是为了防止程序启动时，pika_pose 数据为空的时候，程序执行失败
        self.x, self.y, self.z = self.eff_position[0],self.eff_position[1],self.eff_position[2]
        self.roll, self.pitch, self.yaw = self.eff_position[3],self.eff_position[4],self.eff_position[5]

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
            # 检测到复位，机械臂回到初始点位并且记录 右 坐标原点
            xarm_control.moveL(self.eff_position,mode=0)
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

    # 增量式控制
    def calc_pose_incre(self,base_pose, pose_data):
        begin_matrix = self.tools.xyzrpy2Mat(base_pose[0], base_pose[1], base_pose[2],
                                                    base_pose[3], base_pose[4], base_pose[5])
        zero_matrix = self.tools.xyzrpy2Mat(self.eff_position[0],self.eff_position[1],self.eff_position[2],
                                            self.eff_position[3],self.eff_position[4],self.eff_position[5])
        end_matrix = self.tools.xyzrpy2Mat(pose_data[0], pose_data[1], pose_data[2],
                                                pose_data[3], pose_data[4], pose_data[5])
        result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
        xyzrpy = self.tools.mat2xyzrpy(result_matrix)
        return xyzrpy
    
    # 订阅pika_pose回调函数
    def pose_callback(self,data):
        
        x = data.pose.position.x * 1000  #由m到mm
        y = data.pose.position.y * 1000
        z = data.pose.position.z * 1000
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
                
        self.x,self.y,self.z,   self.roll, self.pitch, self.yaw = self.adjustment(x,y,z,roll,pitch,yaw)
                
    # 调整矩阵函数
    def adjustment(self,x,y,z,Rx,Ry,Rz):
        
        transform = self.tools.xyzrpy2Mat(x,y,z,   Rx, Ry, Rz)

        r_adj = self.tools.xyzrpy2Mat(self.pika_to_arm[0],self.pika_to_arm[1],self.pika_to_arm[2],
                                      self.pika_to_arm[3],self.pika_to_arm[4],self.pika_to_arm[5],)   # 调整坐标轴方向  pika--->机械臂末端
        
        transform = np.dot(transform, r_adj)
        
        x_,y_,z_,Rx_,Ry_,Rz_ = self.tools.mat2xyzrpy(transform)
                
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
                    
            #下发pose至机械臂
            if self.flag:
                xarm_control.moveL(increment_pose,mode=1)
            
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