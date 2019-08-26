#!/usr/bin/python

import rospy
from control_car.msg import keyboard_control
from control_car.msg import watch_tower_control
import Adafruit_PCA9685
import time


class ActuatorNode:	
	servo_port = 1	
	motor_port = 2	
	pwm = None
	left_max = 600
	right_max = 400
	steer_center = (left_max+right_max)/2
	min_speed = 220
	max_speed = 420
	speed_center = (min_speed+max_speed)/2
	steer = steer_center
	speed = speed_center
	jump = 1

	def __init__(self):
		self.pwm = Adafruit_PCA9685.PCA9685()
		self.pwm.set_pwm_freq(60)
 
	def keyboard(self,data): 
		data = data.control

		if (data == ord('a')):
			self.steer -= self.jump*30
			if (self.steer < self.right_max):
				self.steer = self.right_max
			self.pwm.set_pwm(self.servo_port, 0, self.steer)
			time.sleep(0.01)
			print("left turn: ", self.steer)

		if (data == ord('d')):
			self.steer += self.jump*30
			if (self.steer > self.left_max):
				self.steer = self.left_max
			self.pwm.set_pwm(self.servo_port, 0, self.steer) 
			time.sleep(0.01)
			print("right turn: ", self.steer) 

		if (data == ord('w')):
			self.speed += self.jump*2
			if (self.speed > self.max_speed):
				self.speed = self.max_speed
			self.pwm.set_pwm(self.motor_port, 0, self.speed)
			time.sleep(0.01)
			print("forward: ", self.speed)

		if (data == ord('s')): 
			self.speed -= self.jump*2
			if (self.speed < self.min_speed):
				self.speed = self.min_speed
			self.pwm.set_pwm(self.motor_port, 0, self.speed)
			time.sleep(0.01)
			print("slow down: ", self.speed)

		if (data == ord('x')):
			self.speed = self.speed_center
			self.pwm.set_pwm(self.motor_port, 0, self.speed)
			time.sleep(0.01)
			self.steer = self.steer_center
			self.pwm.set_pwm(self.servo_port, 0, self.steer_center) 
			time.sleep(0.01)
       
		if (data == ord('q')):
			self.pwm.set_pwm(self.servo_port, 0, self.right_max) 
			print("sharp left turn") 
			time.sleep(0.01)
	
		if (data == ord('e')):
			self.pwm.set_pwm(self.servo_port, 0, self.left_max) 
			print("sharp right turn") 
			time.sleep(0.01)
	
	def watch_tower(self,data):
		data = data.control
		self.pwm.set_pwm(self.servo_port, 0, data[0]) 
		time.sleep(0.01)
		self.pwm.set_pwm(self.motor_port, 0, data[1]) 
		time.sleep(0.01)
				

def listener(actuatorNode):
	rospy.init_node('custom_listener', anonymous=True)
	rospy.Subscriber("keyboard_control",keyboard_control,actuatorNode.keyboard,queue_size=1)
	rospy.Subscriber("watch_tower_control",watch_tower_control,actuatorNode.watch_tower,queue_size=1)
	rospy.spin()

if __name__=='__main__':
	actuatorNode = ActuatorNode()
	listener(actuatorNode)
