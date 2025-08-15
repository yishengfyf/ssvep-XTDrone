#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机状态发送器 (State Sender)

功能:
1. 作为ROS节点运行
2. 订阅XTDrone中无人机的本地位姿话题 (/mavros/local_position/pose)
3. 将位姿数据 (位置+姿态四元数) 通过UDP协议发送出去
"""

import rospy
from geometry_msgs.msg import PoseStamped
import socket
import struct
import time

# --- UDP 配置 ---
# TODO: 您可以根据需要修改这里的IP和端口
UDP_IP = "172.20.10.225"  # 发送到界面机，应是PC2的IP地址
UDP_PORT = 20002      # 使用一个新的、未被占用的端口

# 创建UDP Socket
# 我们在全局创建一次，避免在回调函数中反复创建
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"无人机状态发送器已启动")
print(f"将通过UDP发送位姿数据到: {UDP_IP}:{UDP_PORT}")

def pose_callback(msg):
    """
    位姿话题的回调函数。
    当收到新的位姿消息时，此函数会被调用。
    """
    # 1. 从ROS消息中提取数据
    # 位置 (x, y, z)
    pos_x = msg.pose.position.x
    pos_y = msg.pose.position.y
    height_offset = 0.5
    pos_z = msg.pose.position.z + height_offset  # 我们需要在Z轴上加一个高度偏移
    
    # 姿态四元数 (x, y, z, w)
    ori_x = msg.pose.orientation.x
    ori_y = msg.pose.orientation.y
    ori_z = msg.pose.orientation.z 
    ori_w = msg.pose.orientation.w
    
    # 打印到终端，方便调试
    print(f"无人机位姿 -> POS:({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f}), ORI:({ori_x:.2f}, {ori_y:.2f}, {ori_z:.2f}, {ori_w:.2f})")
    
    # 2. 将数据打包成字节序列
    # '7f' 表示7个float类型的数据 (4字节浮点数)
    # 顺序: pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w
    try:
        packet = struct.pack('7f', pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w)
        
        # 3. 通过UDP发送数据包
        sock.sendto(packet, (UDP_IP, UDP_PORT))
        
    except Exception as e:
        rospy.logerr(f"打包或发送UDP数据时出错: {e}")
        time.ssleep(1.0)  # 如果发生错误，等待1秒后重试

def main():
    """主函数"""
    # 初始化ROS节点
    rospy.init_node('state_sender_node', anonymous=True)
    
    # 定义要订阅的话题名称
    # TODO: 确认无人机名称是否为'iris_0'
    drone_name = "iris_0"
    pose_topic = f"/{drone_name}/mavros/local_position/pose"
    
    rospy.loginfo(f"正在订阅话题: {pose_topic}")
    
    # 创建订阅者
    rospy.Subscriber(pose_topic, PoseStamped, pose_callback)
    
    # 保持节点运行，直到被关闭
    rospy.spin()
    
    # 关闭时清理Socket
    sock.close()
    print("Socket连接已关闭")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass