#!/usr/bin/env python
# -*- coding : utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : stop
anything else : stop and exit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.0
turn = 0.0
line_code_enabled = False

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def qrCodeInfoCallback(msg):
    info_msg = msg.data
    if info_msg == "Google":
        stopMotion()

def lineCodeInfoCallback(msg):
    global line_code_enabled
    if line_code_enabled:
        info_msg = msg.data
        if info_msg == "L":
            speed = 0.0
            turn = 3.5
            print(vels(speed, turn))
            moveRobot(speed, turn)
        elif info_msg == "R":
            speed = 0.0
            turn = -3.5
            print(vels(speed, turn))
            moveRobot(speed, turn)
        elif info_msg == "F":
            speed = 0.5
            turn = 0.0
            print(vels(speed, turn))
            moveRobot(speed, turn)

def moveRobot(speed, turn):
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = turn
    pub.publish(twist)

def stopMotion():
    global speed, turn
    speed = 0.0
    turn = 0.0
    print(vels(speed, turn))
    moveRobot(speed, turn)

def guiSendCallback(msg):
    global line_code_enabled
    command = msg.data
    if command == "start":
        line_code_enabled = True
    else:
        line_code_enabled = False

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_pub_test3.py')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    qr_info_sub = rospy.Subscriber('/qr_code_info', String, qrCodeInfoCallback)
    line_info_sub = rospy.Subscriber('/line_code_info', String, lineCodeInfoCallback)
    gui_send_sub = rospy.Subscriber('/gui_send', String, guiSendCallback)

    status = 0
    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            if key in ['w', 'W']:
                speed = 0.5
                turn = 0.0
                print(vels(speed, turn))
                moveRobot(speed, turn)
            elif key in ['x', 'X']:
                speed = -0.5
                turn = 0.0
                print(vels(speed, turn))
                moveRobot(speed, turn)
            elif key in ['a', 'A']:
                speed = 0.0
                turn = + 3.5
                print(vels(speed, turn))
                moveRobot(speed, turn)
            elif key in ['d', 'D']:
                speed = 0.0
                turn = -3.5
                print(vels(speed, turn))
                moveRobot(speed, turn)
            elif key in [' ', 's', 'S']:
                stopMotion()
            else:
                if key == '\x03':
                    break

    except Exception as e:
        print(e)

    finally:
        stopMotion()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
