#!/usr/bin/python
import apriltag
import cv2
import numpy as np
import time
import math
import rospy
from control_car.msg import watch_tower_control


left_max = 600
right_max = 400
steer_center = (left_max+right_max)/2
min_speed = 220
max_speed = 420
speed_center = (min_speed+max_speed)/2
steer = steer_center
speed = speed_center

target_x = 0
target_y = 0
target_angle = 0
turning_radius = 0

def set_target(event,x,y,flags,param):
	global target_x
	global target_y
	if event == cv2.EVENT_FLAG_LBUTTON:
		target_x = x
		target_y = y

def main():
	global target_x
	global target_y
	global target_angle
	global turning_radius

	# ROS Publisher Init
	pub = rospy.Publisher('watch_tower_control',watch_tower_control,queue_size=1)
	rospy.init_node('custom_talker', anonymous=True)
	control_signal = watch_tower_control()

	DIM = (800, 600)

	# USB Camera Parameters
	K=np.array([[646.986342788419, 0.0, 383.9135552285603], [0.0, 647.4588452248628, 315.82293891405163], [0.0, 0.0, 1.0]])
	D=np.array([[0.40302428743352114], [0.10116712172043976], [0.6206370706341585], [-4.18627051202362]])
	camera_params = (646.986342788419,647.4588452248628,383.9135552285603,315.82293891405163)	

	video_capture = cv2.VideoCapture(0)
	video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT,600)
	video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,800) 
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
	detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
	tag_size = 160 

	target_x = 300
	target_y = 200
	target_angle = 0
	turning_radius = 30
	print("target point is "+"("+str(target_x)+","+str(target_y)+")")
	print("traget angle is "+str(target_angle))
	print("turning radius is "+str(turning_radius))
	
	cv2.namedWindow('image')
	cv2.setMouseCallback('image', set_target)
	
	while (True):
		_, frame = video_capture.read()
		gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray_frame = cv2.remap(gray_frame, map1, map2, interpolation = cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
		result = detector.detect(gray_frame)
		
		target_right = (np.int(target_x +math.cos(math.radians(target_angle))*turning_radius),np.int(target_y +math.sin(math.radians(target_angle))*turning_radius))
		target_left  = (np.int(target_x -math.cos(math.radians(target_angle))*turning_radius),np.int(target_y -math.sin(math.radians(target_angle))*turning_radius))
		cv2.circle(gray_frame,target_right,turning_radius,(255,255,255),-1)	
		cv2.circle(gray_frame,target_left,turning_radius,(0,0,0),-1)	
		
		if (result): 
			start_point = result[0].center.astype(int)
			start_angle = math.degrees(math.atan2(result[0].corners[1][1]-result[0].corners[0][1],result[0].corners[1][0]-result[0].corners[0][0]))
			curr_right = (np.int(start_point[0]+math.cos(math.radians(start_angle))*turning_radius),np.int(start_point[1]+math.sin(math.radians(start_angle))*turning_radius))
			curr_left = (np.int(start_point[0]-math.cos(math.radians(start_angle))*turning_radius),np.int(start_point[1]-math.sin(math.radians(start_angle))*turning_radius))
			cv2.circle(gray_frame,curr_right,turning_radius,(255,255,255),-1)	
			cv2.circle(gray_frame,curr_left,turning_radius,(0,0,0),-1)	

			dist = np.array([	np.linalg.norm(np.array(target_right)-np.array(curr_right)),
								np.linalg.norm(np.array(target_right)-np.array(curr_left)),
								np.linalg.norm(np.array(target_left)-np.array(curr_right)),
								np.linalg.norm(np.array(target_left)-np.array(curr_left))])
			dist[dist<=2*turning_radius] = 100000
			index = np.argmin(dist)
			if np.linalg.norm(np.array([target_x,target_y])-np.array(start_point)) < 10:
				control_signal.control = [0,speed_center]
			else:
				control_signal.control = [0,max_speed]
			time.sleep(0.01)

			if index == 0:
				dest_angle = math.degrees(math.atan2(target_right[0]-curr_right[0],target_right[1]-curr_right[1]))
				dest_angle = (180-dest_angle) if dest_angle>0 else -180-dest_angle
				pt1 = (np.int(curr_right[0]-math.cos(math.radians(dest_angle))*turning_radius),np.int(curr_right[1]-math.sin(math.radians(dest_angle))*turning_radius))
				pt2 = (np.int(target_right[0]-math.cos(math.radians(dest_angle))*turning_radius),np.int(target_right[1]-math.sin(math.radians(dest_angle))*turning_radius))
				temp_angle = -math.degrees(math.atan2(pt1[0]-pt2[0],pt1[1]-pt2[1]))
				cv2.line(gray_frame,pt1,pt2,(0,0,0),5) 
				cv2.line(gray_frame,target_right,curr_right,(0,0,0),5) 
				if (abs(start_angle-temp_angle)<10):
					control_signal.control[0] = center
					#pwm.set_pwm(1, 0, center)
				else:
					control_signal.control[0] = left_max
					#pwm.set_pwm(1, 0, left_max)
				time.sleep(0.01)
			elif index == 1:
				dest_angle = math.degrees(math.atan2(target_right[0]-curr_left[0],target_right[1]-curr_left[1]))
				dest_angle = (180-dest_angle) if dest_angle>0 else -180-dest_angle
				dest_angle = dest_angle + math.degrees(math.acos(turning_radius/(dist[1]/2)))+90
				pt1 = (np.int(curr_left[0]-math.cos(math.radians(dest_angle))*turning_radius),np.int(curr_left[1]-math.sin(math.radians(dest_angle))*turning_radius))
				pt2 = (np.int(target_right[0]+math.cos(math.radians(dest_angle))*turning_radius),np.int(target_right[1]+math.sin(math.radians(dest_angle))*turning_radius))
				temp_angle = -math.degrees(math.atan2(pt1[0]-pt2[0],pt1[1]-pt2[1]))
				cv2.line(gray_frame,pt1,pt2,(255,255,255),5) 
				cv2.line(gray_frame,target_right,curr_left,(0,0,0),5)
				if (abs(start_angle-temp_angle)<10):
					control_signal.control[0] = center
					#pwm.set_pwm(1, 0, center)
				else:				
					control_signal.control[0] = right_max
					#pwm.set_pwm(1, 0, right_max)
				time.sleep(0.01)
				#print(start_angle,temp_angle)
			elif index == 2:
				dest_angle = math.degrees(math.atan2(target_left[0]-curr_right[0],target_left[1]-curr_right[1]))
				dest_angle = (180-dest_angle) if dest_angle>0 else -180-dest_angle
				dest_angle = dest_angle - math.degrees(math.acos(turning_radius/(dist[2]/2)))+ 90
				pt1 = (np.int(curr_right[0]-math.cos(math.radians(dest_angle))*turning_radius),np.int(curr_right[1]-math.sin(math.radians(dest_angle))*turning_radius))
				pt2 = (np.int(target_left[0]+math.cos(math.radians(dest_angle))*turning_radius),np.int(target_left[1]+math.sin(math.radians(dest_angle))*turning_radius))
				temp_angle = -math.degrees(math.atan2(pt1[0]-pt2[0],pt1[1]-pt2[1]))
				cv2.line(gray_frame,pt1,pt2,(255,255,255),5) 
				cv2.line(gray_frame,target_left,curr_right,(0,0,0),5)
				if (abs(start_angle-temp_angle)<10):
					control_signal.control[0] = center
					#pwm.set_pwm(1, 0, center)
				else:
					control_signal.control[0] = left_max
					#pwm.set_pwm(1, 0, left_max)
				time.sleep(0.01)
				#print(start_angle,temp_angle)
			else:
				dest_angle = math.degrees(math.atan2(target_left[0]-curr_left[0],target_left[1]-curr_left[1]))
				dest_angle = (180-dest_angle) if dest_angle>0 else -180-dest_angle
				pt1 = (np.int(curr_left[0]+math.cos(math.radians(dest_angle))*turning_radius),np.int(curr_left[1]+math.sin(math.radians(dest_angle))*turning_radius))
				pt2 = (np.int(target_left[0]+math.cos(math.radians(dest_angle))*turning_radius),np.int(target_left[1]+math.sin(math.radians(dest_angle))*turning_radius))
				temp_angle = -math.degrees(math.atan2(pt1[0]-pt2[0],pt1[1]-pt2[1]))
				cv2.line(gray_frame,pt1,pt2,(0,0,0),5) 
				cv2.line(gray_frame,target_left,curr_left,(0,0,0),5)
				if (abs(start_angle-temp_angle) < 10):
					control_signal.control[0] = center
					#pwm.set_pwm(1, 0, center)
				else:
					control_signal.control[0] = right_max
					#pwm.set_pwm(1, 0, right_max)
				time.sleep(0.01)
				#print(start_angle,temp_angle)
		else:	
			control_signal.control = [center,400]
			#pwm.set_pwm(2, 0, 400)
		
		pub.publish(control_signal)
		cv2.imshow("image",gray_frame)
		key = cv2.waitKey(1)&0xFF
		if key == ord('q'):
			break	
		elif key == ord('w'):
			if turning_radius < 100:
				turning_radius+=1
		elif key == ord('s'):
			if turning_radius > 10:
				turning_radius-=1
		elif key == ord('a'):
			target_angle -= 5
			if target_angle <= -180:
				target_angle = 180
		elif key == ord('d'):
			target_angle += 5
			if target_angle >= 180:
				target_angle = -180
	video_capture.release()
	cv2.destroyAllWindows()
	control_signal.control = [center,400]
	pub.publish(control_signal)
	#pwm.set_pwm(2, 0, 400)


if __name__=="__main__":
	try:	
		main()
	except rospy.ROSInterruptException:
		pass