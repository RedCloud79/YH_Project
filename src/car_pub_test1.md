```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
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

speed = 0.2
turn = 0.5

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    status = 0
    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            if key in ['w', 'W']:
                speed = speed + 0.1
                print(vels(speed, turn))
            elif key in ['x', 'X']:
                speed = speed - 0.1
                print(vels(speed, turn))
            elif key in ['a', 'A']:
                turn = turn + 0.1
                print(vels(speed, turn))
            elif key in ['d', 'D']:
                turn = turn - 0.1
                print(vels(speed, turn))
            elif key in [' ', 's', 'S']:
                speed = 0
                turn = 0
                print(vels(speed, turn))
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

```
