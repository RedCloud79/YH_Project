```python
#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from sensor_msgs.msg import Imu

def imuCallback(msg):
    # /imu 토픽 메시지 콜백 함수
    global imu_data
    imu_data = msg

def teleopCallback(msg):
    # /teleop 토픽 메시지 콜백 함수
    global teleop_data
    teleop_data = msg

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0

    imu_data = None
    teleop_data = None

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rospy.Subscriber('/imu', Imu, imuCallback)  # /imu 토픽을 구독합니다.
    rospy.Subscriber('/teleop', Twist, teleopCallback)  # /teleop 토픽을 구독합니다.

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        if imu_data is not None and teleop_data is not None:
            # IMU 데이터와 teleop 데이터가 모두 있을 때만 처리합니다.
            dt = (current_time - last_time).to_sec()

            # IMU 데이터에서 필요한 값을 추출합니다.
            imu_vx = imu_data.linear_acceleration.x
            imu_vy = imu_data.linear_acceleration.y
            imu_vth = imu_data.angular_velocity.z

            # teleop 데이터에서 필요한 값을 추출합니다.
            teleop_vx = teleop_data.linear.x
            teleop_vy = teleop_data.linear.y
            teleop_vth = teleop_data.angular.z

            # vx, vy, vth 값을 계산합니다.
            vx = imu_vx + teleop_vx
            vy = imu_vy + teleop_vy
            vth = imu_vth + teleop_vth

            # 로봇의 위치를 계산합니다.
            delta_x = (vx * cos(th) - vy * sin(th)) * dt
            delta_y = (vx * sin(th) + vy * cos(th)) * dt
            delta_th = vth * dt
            x += delta_x
            y += delta_y
            th += delta_th

            # 오도메트리 메시지를 생성합니다.
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # 로봇의 위치를 설정합니다.
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0

            # 로봇의 자세를 설정합니다.
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            # 로봇의 속도를 설정합니다.
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            # 오도메트리 메시지를 발행합니다.
            odom_pub.publish(odom)

            last_time = current_time

        r.sleep()

```
