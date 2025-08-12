#!/usr'bin/env python3
# -*- coding: utf-8 -*-
"""
PC2 (木偶/BCI终端) 综合启动器 

功能:
1. 启动一个隔离的“木偶”Gazebo环境
2. 启动姿态更新器 (接收状态)
3. 启动刺激界面及脑电波模拟的指令发送器
"""
import subprocess
import time

def open_terminal(command):
    full_command = f"bash -c '{command}; exec bash'"
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', full_command])

if __name__ == "__main__":
    commands = [
        # 终端1: 启动隔离的“木偶”仿真世界
        ("cd ~/PX4_Firmware && roslaunch px4 outdoor3_visualizer.launch", 10.0),

        # 终端2: 启动“木偶师”，接收UDP状态并更新姿态
        ("cd ~/XTDrone/bci/scripts && python3 pose_updater.py", 2.0),

        # 终端3: 启动刺激界面及脑电波模拟的指令发送器
        ("cd ~/XTDrone/bci/scripts && python3 bci_main.py", 0.0)
    ]

    print("🚀 正在启动 PC2 (木偶/BCI终端) 环境...")
    for cmd, delay in commands:
        open_terminal(cmd)
        if delay > 0:
            time.sleep(delay)
    print("✅ PC2 (木偶/BCI终端) 所有组件已启动！请在指令发送终端中输入指令。")