#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BUDP协议测试发送器
用于测试bci_budp_control.py的接收功能

功能:
1. 发送标准BUDP协议包
2. 支持所有指令 (0-7)
3. 包含CRC16校验
4. 交互式用户界面
5. 详细的调试输出

"""

import socket
import struct
import time
import threading
from datetime import datetime

class BUDPTestSender:
    def __init__(self, target_ip="127.0.0.1", target_port=20001):
        """
        初始化BUDP测试发送器
        
        Args:
            target_ip: 目标IP地址 (默认本地回环)
            target_port: 目标端口 (默认20001)
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.sequence = 0
        self.system_id = 255  # 测试系统ID
        
        # 创建UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 指令描述映射
        self.command_descriptions = {
            0: "switch to OFFBOARD mode and arm",
            1: "fly upward 2s",
            2: "fly downward 2s", 
            3: "fly forward 2s",
            4: "fly backward 2s",
            5: "fly leftward 2s",
            6: "fly rightward 2s",
            7: "AUTO.LAND"
        }
        
        print(f"BUDP测试发送器初始化完成")
        print(f"目标地址: {target_ip}:{target_port}")

    def calculate_crc16(self, data):
        """
        计算CRC16校验和 (与接收端保持一致)
        使用CRC-16-IBM算法
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def create_budp_packet(self, message_id):
        """
        创建BUDP协议数据包
        
        包格式:
        [起始标记][长度][序列号][系统ID][消息ID][CRC16_LOW][CRC16_HIGH]
        
        Args:
            message_id: 消息ID (0-7)
            
        Returns:
            bytes: 完整的BUDP数据包
        """
        # 构建包头 - 无载荷数据
        start_marker = 0xFE
        length = 0  # 载荷长度为0
        sequence = self.sequence
        system_id = self.system_id
        
        # 包头 (5字节) - 只包含指令ID，无载荷
        header = struct.pack('BBBBB', start_marker, length, sequence, system_id, message_id)
        
        # 完整数据包 (不包含CRC) - 只有包头，无载荷
        packet_without_crc = header
        
        # 计算CRC16
        crc = self.calculate_crc16(packet_without_crc)
        
        # 完整数据包 (包含CRC)
        complete_packet = packet_without_crc + struct.pack('<H', crc)
        
        # 递增序列号
        self.sequence = (self.sequence + 1) % 256
        
        return complete_packet

    def debug_packet(self, packet):
        """
        调试输出数据包详细信息
        """
        print(f"\n数据包详细信息:")
        print(f"   总长度: {len(packet)} 字节")
        print(f"   十六进制: {packet.hex().upper()}")
        
        if len(packet) >= 5:
            start_marker = packet[0]
            length = packet[1]
            sequence = packet[2]
            system_id = packet[3]
            message_id = packet[4]
            
            print(f"   起始标记: 0x{start_marker:02X}")
            print(f"   载荷长度: {length}")
            print(f"   序列号: {sequence}")
            print(f"   系统ID: {system_id}")
            print(f"   消息ID: {message_id}")
            
            if len(packet) >= 7:
                crc_bytes = packet[5:7]
                crc = struct.unpack('<H', crc_bytes)[0]
                print(f"   CRC16: 0x{crc:04X}")

    def send_command(self, message_id, show_debug=True):
        """
        发送BUDP指令
        
        Args:
            message_id: 指令ID (0-7)
            show_debug: 是否显示调试信息
            
        Returns:
            bool: 发送是否成功
        """
        try:
            # 验证指令ID
            if message_id not in range(8):
                print(f"无效指令ID: {message_id} (有效范围: 0-7)")
                return False
            
            # 创建数据包
            packet = self.create_budp_packet(message_id)
            
            # 获取指令描述
            desc = self.command_descriptions.get(message_id, f"未知指令{message_id}")
            
            print(f"\n发送BUDP指令: ID={message_id} ({desc})")
            
            # 显示调试信息
            if show_debug:
                self.debug_packet(packet)
            
            # 发送数据包
            self.sock.sendto(packet, (self.target_ip, self.target_port))
            
            print(f"指令发送成功! 时间: {datetime.now().strftime('%H:%M:%S')}")
            return True
            
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def test_all_commands(self):
        """
        测试所有指令 (0-7)
        """
        print(f"\n开始测试所有BUDP指令...")
        
        for cmd_id in range(8):
            desc = self.command_descriptions.get(cmd_id, f"指令{cmd_id}")
            print(f"\n--- 测试指令 {cmd_id}: {desc} ---")
            
            success = self.send_command(cmd_id, show_debug=False)
            if success:
                print(f"指令 {cmd_id} 发送成功")
            else:
                print(f"指令 {cmd_id} 发送失败")
            
            time.sleep(1)  # 间隔1秒
        
        print(f"\n所有指令测试完成!")

    def interactive_mode(self):
        """
        交互式命令模式
        """
        print(f"\n进入交互模式")
        print(f"指令列表:")
        for cmd_id, desc in self.command_descriptions.items():
            print(f"  {cmd_id}: {desc}")
        print(f"\n特殊命令:")
        print(f"  test: 测试所有指令")
        print(f"  help: 显示帮助信息")
        print(f"  quit: 退出程序")
        print(f"\n输入格式: <指令ID>")
        print(f"例如: 1 (上升指令)")
        
        while True:
            try:
                user_input = input(f"\n请输入指令> ").strip()
                
                if not user_input:
                    continue
                
                if user_input.lower() == 'quit':
                    print(f"退出程序")
                    break
                elif user_input.lower() == 'help':
                    self.show_help()
                    continue
                elif user_input.lower() == 'test':
                    self.test_all_commands()
                    continue
                
                # 解析输入 - 只需要指令ID
                try:
                    cmd_id = int(user_input)
                except ValueError:
                    print(f"无效的指令ID: {user_input}")
                    continue
                
                # 发送指令
                self.send_command(cmd_id)
                
            except KeyboardInterrupt:
                print(f"\n程序被中断，退出")
                break
            except Exception as e:
                print(f"输入处理错误: {e}")

    def show_help(self):
        """显示帮助信息"""
        print(f"\nBUDP测试发送器帮助:")
        print(f"1. 输入指令ID (0-7) 发送对应指令")
        print(f"2. 特殊命令:")
        print(f"   - test: 依次测试所有指令")
        print(f"   - help: 显示此帮助信息")
        print(f"   - quit: 退出程序")
        print(f"3. 指令映射:")
        for cmd_id, desc in self.command_descriptions.items():
            print(f"   {cmd_id}: {desc}")

    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            print(f"Socket连接已关闭")

def main():
    """主函数"""
    print(f"BUDP协议测试发送器启动")
    print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # 获取用户配置
    target_ip = input("请输入目标IP地址 (默认: 127.0.0.1): ").strip()
    if not target_ip:
        target_ip = "127.0.0.1"
    
    target_port = input("请输入目标端口 (默认: 20001): ").strip()
    if not target_port:
        target_port = 20001
    else:
        try:
            target_port = int(target_port)
        except ValueError:
            print("无效端口号，使用默认值 20001")
            target_port = 20001
    
    # 创建发送器
    sender = BUDPTestSender(target_ip, target_port)
    
    try:
        # 直接进入交互模式
        sender.interactive_mode()
            
    except KeyboardInterrupt:
        print(f"\n程序被中断")
    finally:
        sender.close()

if __name__ == "__main__":
    main()
