#!/usr/bin/env python3
import math
import numpy as np
from tf.transformations import (quaternion_from_matrix, euler_from_matrix, 
                                quaternion_from_euler, euler_from_quaternion)

class  MATHTOOLS:
    
    # 从xyzrpy创建旋转矩阵
    def xyzrpy2Mat(self,x, y, z, roll, pitch, yaw):
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
    
    # 将矩阵转换为位姿
    def matrix2Pose(self,transform):
        # 提取平移部分 x, y, z
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        
        # 提取旋转矩阵
        rotation_matrix = transform[:3, :3]
        
        # 将旋转矩阵转换为欧拉角 (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_matrix(rotation_matrix)
        
        return x, y, z, roll, pitch, yaw
    
    def mat2xyzrpy(self,matrix):
        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.asin(-matrix[2, 0])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return [x, y, z, roll, pitch, yaw]
    
    def xyzQuaternion2matrix(x, y, z, qx, qy, qz, qw):
        # 四元数到旋转矩阵：根据标准公式，将四元数分量转换为一个 3x3 的旋转矩阵。
        R = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)]
        ])
        
        # 构造齐次变换矩阵：在右下角补上 1，在第四列填入位置向量 [x, y, z]。
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T

    def rpy_to_rotvec(self,roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        将RPY角度(绕X/Y/Z轴旋转)转换为旋转矢量
        
        参数:
            roll: 绕X轴的旋转角度(弧度)
            pitch: 绕Y轴的旋转角度(弧度)
            yaw: 绕Z轴的旋转角度(弧度)
        
        返回:
            旋转矢量 [rx, ry, rz], 方向表示旋转轴, 模长表示旋转角度(弧度)
        """
        # 构建绕各轴的旋转矩阵
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转矩阵 (顺序: Z-Y-X)
        R = np.dot(R_z, np.dot(R_y, R_x))
        
        # 从旋转矩阵计算旋转矢量
        theta = np.arccos((np.trace(R) - 1) / 2)
        
        if np.abs(theta) < 1e-10:  # 零旋转
            return np.array([0, 0, 0])
        else:
            # 旋转轴
            axis = np.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1]
            ]) / (2 * np.sin(theta))
            
            # 旋转矢量 = 旋转轴 * 旋转角度
            return axis * theta


    def rotvec_to_rpy(self,rotvec: np.ndarray) -> tuple:
        """
        将旋转矢量转换为RPY角度(绕X/Y/Z轴旋转)
        
        参数:
            rotvec: 旋转矢量 [rx, ry, rz]
        
        返回:
            (roll, pitch, yaw): 绕X/Y/Z轴的旋转角度(弧度)
        """
        theta = np.linalg.norm(rotvec)
        
        if np.abs(theta) < 1e-10:  # 零旋转
            return (0, 0, 0)
        
        # 旋转轴
        axis = rotvec / theta
        
        # 构建旋转矩阵 (Rodrigues公式)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        
        # 从旋转矩阵提取RPY (采用Z-Y-X顺序)
        # 参考: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return (roll, pitch, yaw)
    