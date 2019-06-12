#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from std_msgs.msg import Int16
import numpy

import socket

HOST = ''  # Standard loopback interface address (localhost)
PORT = 65432       # Port to listen on (non-privileged ports are > 1023)

def callback():
	    #Setup publisher
    	    rospy.init_node('feedback_msg_socket', anonymous=True)
    	    rate = rospy.Rate(10) # 10hz

	    #Setup socket to read messages sent by UI 
	    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	    s.bind((HOST, PORT))
	    s.listen(1)
	    conn, addr = s.accept()
	    print('Connected by', addr)
	    while True:
		    data = conn.recv(1024)
		    if not data:
		        continue
		    else:
		        print(data)
			ex_id = int (data) #TODO change to 1, 2, or 3
			print ( ex_id)
			pub.publish(ex_id)
			rospy.loginfo (ex_id)
			#rate.sleep()
		        if data == b'1':
		            print("boooo!")
		            conn.sendall(data)

		    '''
		    while not rospy.is_shutdown():
			ex_id = data #TODO change to 1, 2, or 3
			pub.publish(ex_id)
			rospy.loginfo (ex_id)
			rate.sleep()
		    '''

if __name__ == '__main__':
    rospy.Subscriber('feedback', String, self.callback)