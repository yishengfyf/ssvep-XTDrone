#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机姿态更新器 (Pose Updater)

功能:
1. 作为ROS节点运行
2. 在后台线程中接收来自NUC2的UDP位姿数据包
3. 调用Gazebo的ROS服务 (/gazebo/set_model_state)
4. 实时更新PC2(bci界面端)上仿真无人机模型的位姿

"""

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose
import socket
import struct
import threading
import time

# --- UDP 配置 ---
# TODO: 确保这里的端口和 state_sender.py 中的一致
UDP_IP = "0.0.0.0" # 监听所有端口（可以根据需要修改为hostIP）
UDP_PORT = 20002

# --- 全局变量 ---
# 使用线程锁来安全地在线程间共享数据
pose_lock = threading.Lock()
# 初始化一个默认的位姿，避免程序启动时出错
latest_pose = Pose()
latest_pose.position.z = 1.0  # 设置一个合理的默认高度
# latest_pose.orientation.w = 1.0 # 有效的默认四元数
new_pose_received = False

def udp_listener():
    """
    运行在独立线程中的UDP监听器。
    它只负责接收和解析UDP数据，然后更新全局的latest_pose变量。
    """
    global latest_pose, new_pose_received
    
    # 创建并绑定UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    rospy.loginfo(f"UDP监听器已启动，正在监听端口 {UDP_PORT}...")

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(1024)  # 阻塞等待数据
            
            # '7f' 对应7个float, 7 * 4 = 28字节
            if len(data) == 28:
                unpacked_data = struct.unpack('7f', data)
                
                # 创建一个新的Pose对象
                current_pose = Pose()
                current_pose.position.x = unpacked_data[0]
                current_pose.position.y = unpacked_data[1]
                current_pose.position.z = unpacked_data[2]
                current_pose.orientation.x = unpacked_data[3]
                current_pose.orientation.y = unpacked_data[4]
                current_pose.orientation.z = unpacked_data[5]
                current_pose.orientation.w = unpacked_data[6]
                
                # 使用线程锁来安全地更新全局变量
                with pose_lock:
                    latest_pose = current_pose
                    new_pose_received = True
                    
        except Exception as e:
            rospy.logerr(f"UDP接收或解析错误: {e}")

def main():
    """主函数"""
    global latest_pose, new_pose_received

    # 初始化ROS节点
    rospy.init_node('pose_updater_node', anonymous=True)
    
    # 启动UDP监听线程
    udp_thread = threading.Thread(target=udp_listener)
    udp_thread.daemon = True  # 设置为守护线程，主程序退出时它也会退出
    udp_thread.start()

    # 等待Gazebo的服务可用
    rospy.loginfo("正在等待 /gazebo/set_model_state 服务...")
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rospy.loginfo("服务已连接!")
    
    # 定义我们要控制的模型名称和参考系
    # TODO: 确认无人机名称是否为'iris_0'
    model_name = "iris_0"
    reference_frame = "world"  # 或者 "map"

    rate = rospy.Rate(100)  # 设置一个100Hz的循环频率

    while not rospy.is_shutdown():
        # 检查是否有新的位姿数据到达
        if new_pose_received:
            # 创建一个服务请求
            request = SetModelStateRequest()
            request.model_state.model_name = model_name
            request.model_state.reference_frame = reference_frame
            
            # 使用线程锁来安全地读取全局变量
            with pose_lock:
                request.model_state.pose = latest_pose
                new_pose_received = False # 重置标志位
            
            try:
                # 调用服务来更新模型位姿
                response = set_model_state_service(request)
                if not response.success:
                    rospy.logwarn(f"更新模型位姿失败: {response.status_message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"服务调用失败: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass