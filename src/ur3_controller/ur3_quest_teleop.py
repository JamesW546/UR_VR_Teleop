#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rtde_control import RTDEControlInterface as RTDEControl
import numpy as np
import time

# 导入 Quest Reader 库 / Import Quest Reader library
from oculus_reader.reader import OculusReader

class UR3DualQuestController(Node):
    def __init__(self):
        super().__init__('ur3_dual_quest_controller')
        self.get_logger().info('UR3 Quest 双机械臂控制节点已启动')
        # --- 设备初始化 --- / Device initialization
        # IP 地址使用占位符，用户请替换为实际设备 IP
        # Placeholder IPs are used here. Please replace them with actual device IPs
        # For example: robot1_ip = "192.168.1.10"
        # UR 机械臂的 IP（左臂/右臂），请替换为实际机器人 IP
        # IPs of the UR robots (left arm / right arm); replace with the actual robot IPs
        self.robot1_ip = "192.168.XXX.XXX"  # UR 左臂 / UR left arm (replace with real IP)
        self.robot2_ip = "192.168.XXX.XXX"  # UR 右臂 / UR right arm (replace with real IP)
        self.oculus_ip = "192.168.XXX.XXX"  # Quest 设备 IP（替换为实际 IP） / Quest device IP (replace with real IP)

        # 初始化连接对象 / initialize connection objects
        self.rtde_c1 = None
        self.rtde_c2 = None
        self.oculus = None

        # 连接机器人（使用占位 IP，运行前请替换为真实 IP） / Connect to robots (using placeholder IPs; replace before running)
        try:
            self.rtde_c1 = RTDEControl(self.robot1_ip)
            self.get_logger().info(f"（占位）将尝试连接到机械臂1，IP: {self.robot1_ip}")
            self.rtde_c2 = RTDEControl(self.robot2_ip)
            self.get_logger().info(f"（占位）将尝试连接到机械臂2，IP: {self.robot2_ip}")
        except Exception as e:
            self.get_logger().error(f"无法连接到机器人（请检查并替换占位 IP）: {e}")
            # 如果连接失败，则保持 None 并继续，后续 control_loop 会检测并提前返回
            self.rtde_c1 = None
            self.rtde_c2 = None

        # 连接 Quest（使用占位 IP，请替换） / Connect to Quest (using placeholder IP; replace before running)
        try:
            self.oculus = OculusReader(ip_address=self.oculus_ip)
            self.get_logger().info(f"（占位）将尝试连接到 Quest 设备, IP: {self.oculus_ip}")
        except Exception as e:
            self.get_logger().error(f"无法连接到 Quest 设备（请检查并替换占位 IP）: {e}")
            self.oculus = None

        # --- 控制逻辑参数 --- / Control loop parameters
        self.linear_speed_scale = 1  # 位置偏差到速度的缩放比例 / scale from position delta to speed
        self.acceleration = 1       # 机器人加速度 / robot acceleration

        # --- 状态变量 (左右手各一套) --- / State variables (separate for left and right controllers)
        self.is_calibrated_L = False      # 左手是否已校准原点 / whether left controller origin is calibrated
        self.origin_pose_L = None         # 存储左手原点位姿矩阵 / store left origin pose matrix

        self.is_calibrated_R = False      # 右手是否已校准原点 / whether right controller origin is calibrated
        self.origin_pose_R = None         # 存储右手原点位姿矩阵 / store right origin pose matrix

        # --- 主控制循环 --- / Main control loop
        self.timer = self.create_timer(0.01, self.control_loop) # 100Hz 刷新 rate
        self.get_logger().info("等待 Quest 手柄输入...")
        self.get_logger().info("请分别按住左右手柄扳机键来控制对应的机器人。")

    def control_loop(self):
        if not all([self.rtde_c1, self.rtde_c2, self.oculus]):
            return

    # 1. 从 Quest 读取最新数据 / Read latest data from Quest
        poses, buttons = self.oculus.get_transformations_and_buttons()

        if not poses or not buttons or 'l' not in poses or 'r' not in poses:
            return

    # ---------------------------------
    # --- 左手柄 -> 控制机械臂 1 --- / Left controller -> control robot 1
    # ---------------------------------
        if buttons.get('LTr', False): # 如果左扳机按下
            if not self.is_calibrated_L:
                self.origin_pose_L = poses['l']
                self.is_calibrated_L = True
                self.get_logger().info("左扳机按下，机械臂1原点已校准！")
            
            current_pos = poses['l'][:3, 3]
            origin_pos = self.origin_pose_L[:3, 3]
            delta = current_pos - origin_pos
            
            # 轴向映射 (可根据你的机器人安装位置和习惯调整)
            # Axis mapping - adjust according to your robot mounting and conventions
            vx = delta[2] * self.linear_speed_scale * -1
            vy = delta[1] * self.linear_speed_scale * -1
            vz = delta[0] * self.linear_speed_scale * -1
            
            velocity_vector1 = [vx, vy, vz, 0.0, 0.0, 0.0]
            self.rtde_c1.speedL(velocity_vector1, self.acceleration)
        else: # 如果左扳机松开
            if self.is_calibrated_L:
                self.rtde_c1.speedL([0.0]*6, self.acceleration)
                self.is_calibrated_L = False # 重置校准状态
                self.get_logger().info("左扳机松开，机械臂1停止。")

    # ---------------------------------
    # --- 右手柄 -> 控制机械臂 2 --- / Right controller -> control robot 2
    # ---------------------------------
        if buttons.get('RTr', False): # 如果右扳机按下
            if not self.is_calibrated_R:
                self.origin_pose_R = poses['r']
                self.is_calibrated_R = True
                self.get_logger().info("右扳机按下，机械臂2原点已校准！")

            current_pos = poses['r'][:3, 3]
            origin_pos = self.origin_pose_R[:3, 3]
            delta = current_pos - origin_pos
            
            # 轴向映射 / Axis mapping
            vx = delta[2] * self.linear_speed_scale
            vy = delta[1] * self.linear_speed_scale * -1
            vz = delta[0] * self.linear_speed_scale
            
            velocity_vector2 = [vx, vy, vz, 0.0, 0.0, 0.0]
            self.rtde_c2.speedL(velocity_vector2, self.acceleration)
        else: # 如果右扳机松开
            if self.is_calibrated_R:
                self.rtde_c2.speedL([0.0]*6, self.acceleration)
                self.is_calibrated_R = False # 重置校准状态
                self.get_logger().info("右扳机松开，机械臂2停止。")

def main(args=None):
    rclpy.init(args=args)
    controller = UR3DualQuestController()
    
    # 检查所有设备是否成功初始化
    if all([controller.rtde_c1, controller.rtde_c2, controller.oculus]):
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            pass
        finally:
            # 在程序关闭时，确保所有机器人停止运动并断开连接
            controller.get_logger().info("程序关闭，正在停止并断开所有机器人...")
            if controller.rtde_c1:
                controller.rtde_c1.speedL([0.0]*6, 1.0)
                controller.rtde_c1.disconnect()
            if controller.rtde_c2:
                controller.rtde_c2.speedL([0.0]*6, 1.0)
                controller.rtde_c2.disconnect()
            controller.get_logger().info('所有机器人连接已断开。')
            controller.destroy_node()
            rclpy.shutdown()
    else:
        controller.get_logger().error("初始化失败，节点未运行。请检查设备连接。")

if __name__ == '__main__':
    main()
