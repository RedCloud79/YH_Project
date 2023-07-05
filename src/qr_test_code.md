## Qr 인식부분

```python
#!/usr/bin/env python

import cv2
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String

cap = cv2.VideoCapture(0)

allowed_formats = ['QRCODE', 'CODE128', 'EAN13']

rospy.init_node('qr_code_scanner')

info_pub = rospy.Publisher('/qr_code_info', String, queue_size=10)

while True:
    ret, frame = cap.read()

    barcodes = pyzbar.decode(frame)

    for barcode in barcodes:
        barcode_type = barcode.type

        if barcode_type in allowed_formats:
            qr_code_data = barcode.data.decode("utf-8")
            
            if qr_code_data == 'https://www.google.co.kr/':
                info_msg = "Google"
            elif qr_code_data == 'http://naver.com':
                info_msg = "Naver"
            elif qr_code_data == 'https://www.daum.net/':
                info_msg = "Daum"
            else:
                info_msg = "Unknown"
            
            info_pub.publish(info_msg)

    cv2.imshow("QR Code Scanner", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

```

## publisher 부분

```python
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

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

# 원하는 정보를 받기 위한 Subscriber 콜백 함수
def qrCodeInfoCallback(msg):
    info_msg = msg.data
    # 원하는 동작에 따라 처리하면 됩니다.
    if info_msg == "Google":
        speed = 0.0
        turn = 0.0
        print(vels(speed, turn))
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = turn
        pub.publish(twist)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    info_sub = rospy.Subscriber('/qr_code_info', String, qrCodeInfoCallback)

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
            elif key in ['x', 'X']:
                speed = -0.5
                turn = 0.0
                print(vels(speed, turn))
            elif key in ['a', 'A']:
                speed = 0.0
                turn = + 3.5
                print(vels(speed, turn))
            elif key in ['d', 'D']:
                speed = 0.0
                turn = -3.5
                print(vels(speed, turn))
            elif key in [' ', 's', 'S']:
                speed = 0.0
                turn = 0.0
                print(vels(speed, turn))
            else:
                if key == '\x03':
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
