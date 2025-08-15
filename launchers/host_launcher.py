#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NUC2 (主机/数据源) 综合启动器 

功能:
1. 启动PX4 + Gazebo主仿真环境
2. 启动通信桥梁
3. 启动BCI UDP指令控制器 (接收指令)
4. 启动状态反馈发送器 (发送状态)
"""
import subprocess
import time

def open_terminal(command):
    full_command = f"bash -c '{command}; exec bash'"
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', full_command])

if __name__ == "__main__":
    commands = [
        ("cd ~/PX4_Firmware && roslaunch px4 outdoor3.launch", 10.0),

        ("cd ~/XTDrone/communication && python3 multirotor_communication.py iris 0", 2.0),

        ("cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts && python3 budp_controller.py iris vel", 2.0),

        ("cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts && python3 state_sender.py", 0.0)
    ]

    print("🚀 正在启动 NUC2 (主机) 环境...")
    for cmd, delay in commands:
        open_terminal(cmd)
        if delay > 0:
            time.sleep(delay)
    print("✅ NUC2 (主机) 所有组件已启动！它正在等待来自PC2的UDP控制指令。")