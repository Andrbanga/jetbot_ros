#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
def joy_teleop(msg):
    k_speed = 0.5
    if msg.buttons[7] == 1:
        k_speed = 1
    cmd_vel = Twist()
    cmd_vel.angular.x = msg.axes[0] * k_speed
    cmd_vel.linear.x = msg.axes[3] * k_speed

    vel_pub.publish(cmd_vel)

# initialization
if __name__ == '__main__':

	# setup ros node
	rospy.init_node('jetbot_joy_teleop')
	rospy.Subscriber('joy', Joy, joy_teleop)
    
	# start running
	rospy.spin()

