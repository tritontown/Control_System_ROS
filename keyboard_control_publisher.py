#!/usr/bin/python
import rospy
from control_car.msg import keyboard_control
import curses
	
def main():
	print("publisher starts")
	screen = curses.initscr()
	curses.noecho()
	curses.cbreak()
	screen.keypad(True)
	
	# ROS Publisher Init
	pub = rospy.Publisher('keyboard_control',keyboard_control,queue_size=1)
	rospy.init_node('custom_talker', anonymous=True)

	control_signal = keyboard_control()
	while(True):
		control_signal.control = screen.getch()
		print(control_signal.control)
		pub.publish(control_signal)	


if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass