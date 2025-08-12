import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
import tty, termios
import socket
import struct
import threading
import time

msg = """
Control Your BCI_Drone
---------------------------
BUDP指令映射:
0 : switch to OFFBOARD mode and arm
1 : fly upward 2s
2 : fly downward 2s
3 : fly forward 2s
4 : fly backward 2s
5 : fly leftward 2s
6 : fly rightward 2s
7 : AUTO.LAND
CTRL-C to quit
"""

# BUDP协议相关全局变量
budp_enabled = False
budp_sock = None
budp_port = 20001
latest_budp_command = None
command_buffer = []

# 飞行状态标志 - 安全机制
flight_mode_flag = 0  # 0: 地面模式(可起飞), 1: 飞行模式(禁止起飞)

# BUDP指令映射
budp_command_mapping = {
    0: '0',  # switch to OFFBOARD mode and arm
    1: '1',  # fly upward 2s
    2: '2',  # fly downward 2s
    3: '3',  # fly forward 2s
    4: '4',  # fly backward 2s
    5: '5',  # fly leftward 2s
    6: '6',  # fly rightward 2s
    7: '7'   # AUTO.LAND
}

def calculate_crc16(data):
    """计算CRC16校验和"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def parse_budp_packet(data):
    """解析BUDP协议包"""
    if len(data) < 7:
        return None
    
    try:
        # 解析包头
        start_marker = data[0]      # 0xFE
        length = data[1]            # 载荷长度
        sequence = data[2]          # 序列号
        system_id = data[3]         # 系统ID
        message_id = data[4]        # 消息ID
        
        # 检查起始标记
        if start_marker != 0xFE:
            return None
        
        # 检查数据长度
        if len(data) < 7 + length:
            return None
        
        # 提取载荷
        payload = data[5:5+length] if length > 0 else b''
        
        # 提取并验证CRC
        if len(data) >= 7 + length:
            received_crc = struct.unpack('<H', data[5+length:7+length])[0]
            calculated_crc = calculate_crc16(data[:5+length])
            
            if received_crc != calculated_crc:
                return None
    
        # 解析载荷参数
        param = None
        if len(payload) >= 4:
            try:
                param = struct.unpack('<f', payload[:4])[0]
            except:
                pass
        
        return {
            'message_id': message_id,
            'param': param,
            'sequence': sequence,
            'system_id': system_id
        }
        
    except Exception as e:
        return None

def budp_listener():
    """BUDP协议监听线程"""
    global latest_budp_command, command_buffer, budp_sock, budp_enabled
    
    try:
        budp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        budp_sock.bind(('', budp_port))
        budp_sock.settimeout(0.1)  # 非阻塞接收
        print(f"BUDP监听启动，端口: {budp_port}")
        
        while budp_enabled:
            try:
                data, addr = budp_sock.recvfrom(1024)
                parsed = parse_budp_packet(data)
                
                if parsed:
                    message_id = parsed['message_id']
                    param = parsed['param']
                    
                    print(f"BUDP指令: ID={message_id}, 参数={param}")
                    
                    # 将BUDP指令映射为键盘指令
                    if message_id in budp_command_mapping:
                        mapped_key = budp_command_mapping[message_id]
                        
                        # 根据指令ID显示对应的动作描述
                        action_descriptions = {
                            0: "switch to OFFBOARD mode and arm",
                            1: "fly upward 2s", 
                            2: "fly downward 2s",
                            3: "fly forward 2s",
                            4: "fly backward 2s", 
                            5: "fly leftward 2s",
                            6: "fly rightward 2s",
                            7: "AUTO.LAND"
                        }
                        
                        # 添加到指令缓冲区
                        command_buffer.append(mapped_key)
                        action_desc = action_descriptions.get(message_id, f"指令{message_id}")
                        print(f"映射指令: {message_id} -> '{mapped_key}' ({action_desc})")
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"BUDP接收错误: {e}")
                
    except Exception as e:
        print(f"BUDP监听器启动失败: {e}")
    finally:
        if budp_sock:
            budp_sock.close()

def getKey():
    """从BUDP指令缓冲区获取指令"""
    global command_buffer
    
    # 从BUDP指令缓冲区获取指令
    if command_buffer:
        key = command_buffer.pop(0)
        return key
    
    # 如果没有BUDP指令，返回空字符串
    return ''

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = sys.argv[1]
    control_type = sys.argv[2]

    rospy.init_node(multirotor_type + '_keyboard_control')

    if control_type != 'vel':
        rospy.logerr("This script only supports velocity control mode ('vel').")
        sys.exit(1)

    cmd_vel_flu_pub = rospy.Publisher('/xtdrone/' + multirotor_type + '_0/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/' + multirotor_type + '_0/cmd', String, queue_size=1)
    twist = Twist()

    # 启动BUDP监听器
    budp_enabled = True
    budp_thread = threading.Thread(target=budp_listener)
    budp_thread.daemon = True
    budp_thread.start()

    print(msg)
    print("\nBUDP监听器已启动，等待指令...")
    
    while not rospy.is_shutdown():
        # 从BUDP指令缓冲区获取指令
        key = getKey()
        
        if not key:  # 如果没有指令，短暂延时后继续循环
            rospy.sleep(0.1)
            continue
            
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        if key == '0':
            # 安全检查：只有在地面模式(flag=0)时才允许起飞
            if flight_mode_flag == 0:
                print("地面模式 - 执行起飞程序")
                print("Switching to OFFBOARD mode and arming")
                cmd_pub.publish(String("OFFBOARD"))
                rospy.sleep(0.1)  # Brief delay to ensure OFFBOARD is processed
                cmd_pub.publish(String("ARM"))
                
                # 起飞后切换到飞行模式
                flight_mode_flag = 1
                print(msg)

                twist.linear.z = 1.0  # Fly upward at 1 m/s
                cmd_vel_flu_pub.publish(twist)
                rospy.sleep(7.0)
                twist.linear.z = 0.0  # Stop
                cmd_vel_flu_pub.publish(twist)
                # print("起飞完成，进入悬停状态")
            else:
                print("飞行模式 - 起飞指令被阻止")

        elif key == '1':
            twist.linear.z = 0.5  # Fly upward at 0.5 m/s
            print("Flying upward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.z = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)

        elif key == '2':
            twist.linear.z = -0.5  # Fly downward at 0.5 m/s
            print("Flying downward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.z = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)

        elif key == '3':
            twist.linear.x = 1.0  # Fly forward at 1 m/s
            print("Flying forward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.x = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)

        elif key == '4':
            twist.linear.x = -1.0  # Fly backward at 1 m/s
            print("Flying backward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.x = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)
        
        elif key == '5':
            twist.linear.y = 1.0  # Fly leftward at 1 m/s
            print("Flying leftward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.y = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)

        elif key == '6':
            twist.linear.y = -1.0  # Fly rightward at 1 m/s
            print("Flying rightward 2s")
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(2.0)  # Fly for 2 second
            twist.linear.y = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)
            print("HOVER")
            print(msg)
        
        elif key == '7':
            cmd_pub.publish(String("AUTO.LAND"))

            # 降落后切换到地面模式
            flight_mode_flag = 0
            print("已切换到地面模式 - 起飞指令已启用")
            print("AUTO.LAND")

        elif key == '\x03':  # CTRL-C
            break

    # 清理资源
    budp_enabled = False
    print("\n 停止BUDP控制器")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)