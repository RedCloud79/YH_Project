```python
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    linear_velocity = data.twist.twist.linear
    angular_velocity = data.twist.twist.angular

    # 위치 정보 출력 예시
    rospy.loginfo("Position - x: {}, y: {}, z: {}".format(position.x, position.y, position.z))
    rospy.loginfo("Orientation - x: {}, y: {}, z: {}, w: {}".format(
        orientation.x, orientation.y, orientation.z, orientation.w))
    rospy.loginfo("Linear Velocity - x: {}, y: {}, z: {}".format(
        linear_velocity.x, linear_velocity.y, linear_velocity.z))
    rospy.loginfo("Angular Velocity - x: {}, y: {}, z: {}".format(
        angular_velocity.x, angular_velocity.y, angular_velocity.z))

if __name__ == '__main__':
    rospy.init_node('turtlebot3_odom_listener')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

```
