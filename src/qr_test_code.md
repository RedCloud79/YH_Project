## Qr 인식부분

```python
import cv2
from pyzbar import pyzbar
import rospy
from std_msgs.msg import String

# 웹캠으로부터 영상 캡처
cap = cv2.VideoCapture(0)  # 웹캠 인덱스를 적절한 값으로 변경해주세요.

# 인식할 큐알 코드 유형 설정
allowed_formats = ['QRCODE', 'CODE128', 'EAN13']  # 인식할 큐알 코드 유형을 적절히 선택해주세요.

# ROS 노드 초기화
rospy.init_node('qr_code_scanner')

# 원하는 정보를 보내기 위해 ROS publisher를 생성
info_pub = rospy.Publisher('/qr_code_info', String, queue_size=10)

while True:
    # 영상 캡처
    ret, frame = cap.read()

    # QR 코드 인식
    barcodes = pyzbar.decode(frame)

    for barcode in barcodes:
        barcode_type = barcode.type

        # 인식할 큐알 코드 유형인지 확인
        if barcode_type in allowed_formats:
            # QR 코드의 내용 추출
            qr_code_data = barcode.data.decode("utf-8")
            
            # 원하는 QR 코드의 내용에 따라서 정보를 설정
            if qr_code_data == 'https://www.google.co.kr/':
                info_msg = "Google"
            elif qr_code_data == 'http://naver.com':
                info_msg = "Naver"
            elif qr_code_data == 'https://www.daum.net/':
                info_msg = "Daum"
            else:
                info_msg = "Unknown"
            
            # 정보를 publisher를 통해 발행
            info_pub.publish(info_msg)

    # 화면에 영상 출력
    cv2.imshow("QR Code Scanner", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()

```

## publisher 부분

```python
#!/usr/bin/env python

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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    info_pub = rospy.Publisher('/qr_code_info', String, queue_size=10)

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

            # publisher를 통해 원하는 정보 보내기
            if key in ['w', 'W', 'x', 'X', 'a', 'A', 'd', 'D', ' ', 's', 'S']:
                info_msg = "Key: " + key
                info_pub.publish(info_msg)

                # 추가한 부분: 원하는 정보가 "Key: Google" 인 경우 움직임을 멈추기
                if info_msg == "Key: Google":
                    speed = 0.0
                    turn = 0.0
                    print(vels(speed, turn))
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
