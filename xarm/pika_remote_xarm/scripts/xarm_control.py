#!/usr/bin/env python3
import rospy
import time
from xarm_msgs.srv import Move,MoveRequest,SetInt16,SetInt16Request,SetAxis,SetAxisRequest   #XARM机械臂

from std_srvs.srv import Trigger

class XARM:
    def __init__(self):
        self.movL_client = rospy.ServiceProxy('/ufactory/move_line', Move)
        self.movJ_client = rospy.ServiceProxy('/ufactory/move_joint', Move)
        self.set_mode_client = rospy.ServiceProxy('/ufactory/set_mode', SetInt16)
        self.set_state_client = rospy.ServiceProxy('/ufactory/set_state', SetInt16)
        
        self.set_motion_ctrl_client = rospy.ServiceProxy('/ufactory/motion_ctrl', SetAxis)
    
    # 机械臂控制函数，mode 0 速度慢，mode 1为快速运动
    def moveL(self,pose,mode=3):
        rospy.wait_for_service('/ufactory/move_line')
        if mode == 0:
            try:
                # 构造请求
                request = MoveRequest()
                request.pose = (pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])  # 示例笛卡尔坐标(x, y, z, roll, pitch, yaw)

                request.mvvelo = 500  # 最大速度
                request.mvacc = 300  # 最大加速度
                request.mvtime = 0.0  # 坐标系指示器
                request.mvradii = 0.0  # 路径混合半径

                # 发送请求  
                response = self.movL_client(request)

                # 处理响应
                if response.ret == 0:
                    rospy.loginfo('Move line successful: %s', response.message)
                else:
                    rospy.logerr('Move line failed with error: %s', response.message)

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
            
        if mode==1:
            try:
                # 构造请求
                request = MoveRequest()
                request.pose = (pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])  # 示例笛卡尔坐标(x, y, z, roll, pitch, yaw)
                request.mvvelo = 2000  # 最大速度
                request.mvacc = 2000  # 最大加速度
                request.mvtime = 0.0  # 坐标系指示器
                request.mvradii = 0.0  # 路径混合半径

                # 发送请求  
                response = self.movL_client(request)

                # 处理响应
                if response.ret == 0:
                    rospy.loginfo('Move line successful: %s', response.message)
                else:
                    rospy.logerr('Move line failed with error: %s', response.message)

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        
        if mode != 0 and mode != 1:
            print("mode error")
            
    def moveJ(self,j1,j2,j3,j4,j5,j6):
        rospy.wait_for_service('/ufactory/move_joint')
        try:
            # 构造请求
            request = MoveRequest()
            request.pose = [j1,j2,j3,j4,j5,j6]  # 示例关节空间坐标(j1, j2, j3, j4, j5, j6)
            request.mvvelo = 0.35  # 最大速度
            request.mvacc = 0.15   # 最大加速度
            request.mvtime = 0.0  # 坐标系指示器
            request.mvradii = 0.0  # 路径混合半径

            # 发送请求  
            response = self.movJ_client(request)

            # 处理响应
            if response.ret == 0:
                rospy.loginfo('Move joint successful: %s', response.message)
            else:
                rospy.logerr('Move joint failed with error: %s', response.message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def degrees_to_radians(self,Rx,Ry,Rz):
        """
        Convert degrees to radians for each component.
        
        Args:
        degrees (tuple): A tuple containing the degrees for Rx, Ry, Rz respectively.

        Returns:
        tuple: A tuple containing the radians for Rx, Ry, Rz respectively.
        """
        import math
        Rx_ = Rx / 180 * math.pi
        Ry_ = Ry / 180 * math.pi
        Rz_ = Rz / 180 * math.pi
        return Rx_, Ry_, Rz_

    def set_motion_ctrl(self,id,data):
        rospy.wait_for_service('/ufactory/motion_ctrl')
        try:
            request = SetAxisRequest()
            request.id = id
            request.data = data
            response = self.set_motion_ctrl_client(request)
            if response.ret == 0:
                rospy.loginfo(f"Set mode to {data} successful: {response.message}")
            else:
                rospy.logerr(f"Set mode to {data} failed with error code {response.ret}: {response.message}")
            return response.ret, response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return -1, str(e)
            
            
    def set_mode(self, mode):
        rospy.wait_for_service("/ufactory/set_mode")
        try:
            request = SetInt16Request()
            request.data = mode
            response = self.set_mode_client(request)
            if response.ret == 0:
                rospy.loginfo(f"Set mode to {mode} successful: {response.message}")
            else:
                rospy.logerr(f"Set mode to {mode} failed with error code {response.ret}: {response.message}")
            return response.ret, response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return -1, str(e)

    def set_state(self, state):
        rospy.wait_for_service("/ufactory/set_state")
        try:
            request = SetInt16Request()
            request.data = state
            response = self.set_state_client(request)
            if response.ret == 0:
                rospy.loginfo(f"Set state to {state} successful: {response.message}")
            else:
                rospy.logerr(f"Set state to {state} failed with error code {response.ret}: {response.message}")
            return response.ret, response.message
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return -1, str(e)

# test code
if __name__ == '__main__':
    # rospy.init_node('xarm_control_node', anonymous=True) # Initialize node if running standalone
    xarm = XARM() 

    # Example usage (uncomment and run in a ROS environment with xArm services available):
    # print("Setting mode to 7 (Cartesian Online)")
    # ret_mode, msg_mode = xarm.set_mode(7)
    # print(f"Set Mode Result: ret={ret_mode}, message='{msg_mode}'")
    # time.sleep(1) # Give some time for mode change
    # print("Setting state to 0 (Start)")
    # ret_state, msg_state = xarm.set_state(0)
    # print(f"Set State Result: ret={ret_state}, message='{msg_state}'")
    # time.sleep(1)

    # Example move command (requires mode and state to be set appropriately first)
    # xarm.moveL([223.778564, 0.357308, 285.864502, -0.390661, 1.521898, -0.379501], mode=0)
    
    # Keep the node running if needed for external calls
    # print("XArm control node ready.")
    # rospy.spin()
