import rtde_control
import rtde_receive
import numpy as np

class URCONTROL:
    def __init__(self,robot_ip):
        # Connect to the robot
        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not self.rtde_c.isConnected():
            print("Failed to connect to the robot control interface.")
            return
        if not self.rtde_r.isConnected():
            print("Failed to connect to the robot receive interface.")
            return
        print("Connected to the robot.")
            
        # Define servoL parameters
        self.speed = 0.15  # m/s
        self.acceleration = 0.1  # m/s^2
        self.dt = 1.0/50  # dt for 500Hz, or 1.0/125 for 125Hz
        self.lookahead_time = 0.1  # s
        self.gain = 300  # proportional gain
        
    def sevol_l(self, target_pose):
        self.rtde_c.servoL(target_pose, self.speed, self.acceleration, self.dt, self.lookahead_time, self.gain)
        
    def get_tcp_pose(self):
        return self.rtde_r.getActualTCPPose()
    

# if __name__ == "__main__":
#     ur = URCONTROL()
#     target_pose = [0.437, -0.1, 0.846, -0.11019068574221307, 1.59479642933605, 0.07061926626169934]
    
#     ur.sevol_l(target_pose)

