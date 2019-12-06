#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# set motor speed 
def set_speed(motor_ID, value):
	# init max pwm to Adafruit motor driver
	max_pwm = 115.0
	# convert velocity val to pwm speed
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
	# choose motor
	if motor_ID == 2:
		motor = motor_left
	elif motor_ID == 1:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	# set motor speed
	motor.setSpeed(speed)
	# choose motor direction
	if value < 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)

# simple kinematic to convert robot cmd_vel to each motor velocity
def on_cmd_vel(cmd_vel):
    left_motor_vel = cmd_vel.linear.x + cmd_vel.angular.x
    right_motor_vel = cmd_vel.linear.x - cmd_vel.angular.x
    set_speed(motor_left_ID,  left_motor_vel)
    set_speed(motor_right_ID,  right_motor_vel)
    if cmd_vel.linear.x == 0 and cmd_vel.angular.x == 0:
        all_stop()

# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)
	# init motor id
	motor_left_ID = 1
	motor_right_ID = 2
	# init motors
	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_joy_teleop')
	# init subscriber
	rospy.Subscriber('cmd_vel', Twist, on_cmd_vel)


	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()

