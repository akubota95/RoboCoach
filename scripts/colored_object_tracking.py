#!/usr/bin/env python

"""
Filename:    colored_object_tracking.py
Authors:     Maryam Pourebadi, Alyssa Kubota, Yi Peng
Description: Handles the Interactive Ecercise module of the RoboCoach
	     to detect and track the green and pink objects in the live videos,
	     draw positions of the objects on the display as they move,
	     and published the coordinates of the center of each tracked object.
    	     Written as part of CSE 276D Healthcare Robotics final project: RoboCoach.
Notes:       Adapted from Gaitech ROS tutorial by Purdue SMART lab
	     https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision
"""

#Color lower and higher range source: #http://www.workwithcolor.com/orange-brown-color-hue-range-01.htm
#object tracking source: https://github.com/SMARTlab-Purdue/ros-tutorial-robot-control-vision


import cv2
import numpy as np
import rospy
#from std_msgs.msg import Float32
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from collections import deque
import argparse
import imutils
import sys
import numpy

#The talker function will run =the webcam. 
def talker():
	rospy.init_node('colored_object_tracking', anonymous=True)
	pub=rospy.Publisher('coordinates',numpy_msg(Floats),queue_size=1)
	rate = rospy.Rate(20) # 10hz
        ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video", dest="/home/turtlebot/turtle_ws_2/src/robocoach/scripts/demo.mp4",help="path")
	ap.add_argument("-b", "--buffer", type=int, default=64,help="max buffer size")
	args = vars(ap.parse_args())
	
	#The talker() will detect any object whose color value ranges between Lower and Upper. 

	#Green
	#low: 85	107	47	
	#high: 178	236	93

	#Red
	#R: 185 G: 29 B: 68
	#R: 190 G: 25 B: 59
	#R: 180 G: 25 B: 59
	#low: 112	28	28
	#high: 255	105	97

	#red-pink
	#low: 196	30	58		
	#high: 227	38	54	


	#blue
	#low:	0	35	102
	#high: 204	204	255

	#Orange yellow
	#low: 150	113	23
	#high: 250	240	190

	#yellow
	#low: 181	166	66
	#high: 255	255	49

	#orange-brown
	#low: 204	85	0
	#high: 255	179	71

	#Pink
	#low: 86	3	25
	#high: 255	192	203

	#pink-red
	#low: 101	0	11
	#high: 255	193	204
		
	#light pink'
	#low: 97	64	81
	#high: 251	204	231

	greenLower = (47, 107, 85)
	greenUpper = (93, 236, 178)
	pts_green = deque(maxlen=64)

	pinkLower = (81, 64, 97)
	pinkUpper = (231, 204, 251)
	pts_pink = deque(maxlen=64)

	if not args.get("video", False):
		camera = cv2.VideoCapture(0)
	else:
		camera = cv2.VideoCapture('args["video"]')
	
	#The object would be detected and a circular contour would be placed over each frame.
	while not rospy.is_shutdown():
		(grabbed, frame) = camera.read()
		if args.get("video") and not grabbed:
			break
		frame = imutils.resize(frame, width=600)
		
		hsv_green = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask_green = cv2.inRange(hsv_green, greenLower, greenUpper)
		mask_green = cv2.erode(mask_green, None, iterations=2)
		mask_green = cv2.dilate(mask_green, None, iterations=2)
		cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center_green = None
		#The centroid coordinate of the green tracker of the circular contour would then be published using coordinates topic.
		if len(cnts_green) > 0:
			c_green = max(cnts_green, key=cv2.contourArea)
			((x_green, y_green), radius_green) = cv2.minEnclosingCircle(c_green)
			M = cv2.moments(c_green)
			center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print(center_green)
			pub.publish(numpy.array([11, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), radius_green], dtype=numpy.float32))	
			rospy.loginfo(numpy.array([11, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), radius_green], dtype=numpy.float32))
			rate.sleep()
			if radius_green > 10:
				cv2.circle(frame, (int(x_green), int(y_green)), int(radius_green),(0, 239, 255), 2)
				cv2.circle(frame, center_green, 5, (0, 239, 255), -1)
		pts_green.appendleft(center_green)
		for i in xrange(1, len(pts_green)):
			if pts_green[i - 1] is None or pts_green[i] is None:
				continue
			thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
			cv2.line(frame, pts_green[i - 1], pts_green[i], (0, 128, 0), thickness)
		
		hsv_pink = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask_pink = cv2.inRange(hsv_pink, pinkLower, pinkUpper)
		mask_pink = cv2.erode(mask_pink, None, iterations=2)
		mask_pink = cv2.dilate(mask_pink, None, iterations=2)
		cnts_pink = cv2.findContours(mask_pink.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center_pink = None

		#The centroid coordinates of the pink tracker of the circular contour would then be published using coordinates topic.
		if len(cnts_pink) > 0:
			c_pink = max(cnts_pink, key=cv2.contourArea)
			((x_pink, y_pink), radius_pink) = cv2.minEnclosingCircle(c_pink)
			M = cv2.moments(c_pink)
			center_pink = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			#print(center_pink)
			pub.publish(numpy.array([22, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), radius_pink], dtype=numpy.float32))	
			rospy.loginfo(numpy.array([22, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), radius_pink], dtype=numpy.float32))
			rate.sleep()
			if radius_pink > 10:
				cv2.circle(frame, (int(x_pink), int(y_pink)), int(radius_pink),(0, 239, 255), 2)
				cv2.circle(frame, center_pink, 5, (0, 239, 255), -1)
		pts_pink.appendleft(center_pink)
		for i in xrange(1, len(pts_pink)):
			if pts_pink[i - 1] is None or pts_pink[i] is None:
				continue
			thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
			cv2.line(frame, pts_pink[i - 1], pts_pink[i], (93, 11, 227), thickness) 

		cv2.imshow("Frame", frame)
		if cv2.waitKey(1) & 0xFF==ord('q'):
			break

	rospy.spin()



# initiate the script and call a talker() function that is responsible for publishing angle values. Also when the node is stopped using CTRL+C the camera would be released by OpenCV and all the window frames would be destroyed, until then the talker function would be run in a loop.

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
	video_capture.release()
	cv2.destroyAllWindows()
	pass
