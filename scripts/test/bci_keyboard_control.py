import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
import tty, termios

msg = """
Control Your XTDrone!
---------------------------
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

# 飞行状态标志 - 安全机制
flight_mode_flag = 0  # 0: 地面模式(可起飞), 1: 飞行模式(禁止起飞)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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

    print(msg)
    while not rospy.is_shutdown():
        key = getKey()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        if key == '0':
            print("Switching to OFFBOARD mode and arming")
            cmd_pub.publish(String("OFFBOARD"))
            rospy.sleep(0.1)  # Brief delay to ensure OFFBOARD is processed
            cmd_pub.publish(String("ARM"))

            flight_mode_flag = 1
            print(msg)

            twist.linear.z = 1.0  # Fly upward at 1 m/s
            cmd_vel_flu_pub.publish(twist)
            rospy.sleep(7.0)
            twist.linear.z = 0.0  # Stop
            cmd_vel_flu_pub.publish(twist)

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
            flight_mode_flag = 0
            print("AUTO.LAND")

        elif key == '\x03':  # CTRL-C
            break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)