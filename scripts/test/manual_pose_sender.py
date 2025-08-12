#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动姿态指令发送器 (Manual Pose Sender)

功能:
1. 用于独立测试 pose_updater_node.py
2. 发送一系列预设的、简单的位姿指令
3. 帮助验证“木偶”无人机是否能被UDP指令控制
"""

import socket
import struct
import time

# --- UDP 配置 ---
# 确保IP和端口与 pose_updater_node.py 中的完全一致
UDP_IP = "127.0.0.1"  # 发送到本机
UDP_PORT = 20002      # 目标端口

# 创建UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("🚀 手动姿态指令发送器已启动")
print(f"📡 将发送指令到: {UDP_IP}:{UDP_PORT}")

# --- 预设的位姿指令序列 ---
# 格式: (x, y, z, "描述")
pose_commands = [
    (0, 0, 3, "移动到场地中心，高度3米"),
    (5, 0, 3, "向前移动到 (5, 0, 3)"),
    (5, 5, 3, "向左移动到 (5, 5, 3)"),
    (0, 5, 3, "向后移动到 (0, 5, 3)"),
    (0, 0, 3, "回到场地中心"),
    (0, 0, 1, "降落到高度1米"),
]

# 默认姿态 (四元数)，保持水平
ori_x, ori_y, ori_z, ori_w = 0.0, 0.0, 0.0, 1.0

try:
    for x, y, z, desc in pose_commands:
        print(f"\n🚁 发送指令: {desc}")
        
        # 将7个浮点数打包成字节序列
        packet = struct.pack('7f', x, y, z, ori_x, ori_y, ori_z, ori_w)
        
        # 发送数据包
        sock.sendto(packet, (UDP_IP, UDP_PORT))
        print(f"✅ 指令已发送 -> POS:({x}, {y}, {z})")
        
        # 等待3秒，给我们足够的时间观察无人机的移动
        time.sleep(3)

    print("\n🏁 所有测试指令已发送完毕。")

except Exception as e:
    print(f"❌ 发送失败: {e}")
finally:
    sock.close()
    print("🔌 Socket连接已关闭")