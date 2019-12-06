#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# init ros publisher to publish velocity 
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# init enable moving flag
enable_mv_flag = False

# func that convert joy msg to velocity
def joy_teleop(msg):
    # coeff to control speed
    k_speed = 0.5
    # if press "X" enable moving
    if msg.buttons[0] == 1:
        enable_mv_flag = True
    # if press "O" disable moving    
    if msg.buttons[1] == 1:
        enable_mv_flag = False
    # while "R1" button is pressed enable speed boost
    if msg.buttons[7] == 1:
        k_speed = 1
    # if enable mv flag true, convert joy to vel and publish it
    if enable_mv_flag:
        # init cmd_vel msg use Twist type which contain angular and linear vel
        cmd_vel = Twist()
        # convert left stick value to angular vel
        cmd_vel.angular.x = msg.axes[0] * k_speed
        # convert right stick value to linear vel
        cmd_vel.linear.x = msg.axes[3] * k_speed
        # publish cmd_vel 
        vel_pub.publish(cmd_vel)
    # else publish 0 vel to cmd_vel
    else:
        cmd_vel = Twist()
        vel_pub.publish(cmd_vel)

# initialization
if __name__ == '__main__':

	# setup ros node
	rospy.init_node('jetbot_joy_teleop')
    # init subscriber to recieve joy msg
	rospy.Subscriber('joy', Joy, joy_teleop)
    
	# start running
	rospy.spin()

