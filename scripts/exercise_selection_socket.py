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
PORT = 65439       # Port to listen on (non-privileged ports are > 1023)



class selection_node(object):
        def __init__(self):
		self.feedback_msg = ""
		
	def talker(self):
		    flag = 0
		    #Setup publisher
	    	    pub=rospy.Publisher('exercises', Int16, queue_size=10)
	    	    rospy.init_node('exercise_selection_socket', anonymous=True)
	    	    rate = rospy.Rate(10) # 10hz

		    #Setup socket to read messages sent by UI 
		    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		    s.bind((HOST, PORT))
		    s.listen(1)
		    conn, addr = s.accept()
		    conn.setblocking(1)
		    print('Connected by', addr)
		    while True:
			    data = conn.recv(1024)
			    if not data:
				continue
			    else:
				data_1 = data.strip()
				ex_id = int (data_1) #TODO change to 1, 2, or 3
				pub.publish(ex_id)
				rospy.loginfo (ex_id)
				#conn.sendall(b'0')
				#rate.sleep()
				if data_1 == b'1':
					msg = "Well done with Exercise 1.\n That was your first attempt."
				elif data_1 == b'2' and flag == 0:
					msg = "Move your right arm more during Exercise 2.\n That was your first attempt."
					flag = 1
				elif data_1 == b'2' and flag != 0:
					msg= "Well done with Exercise 2.\n Your performance has improved!"
				elif data_1 == b'3':
					msg = "Well done with Exercise 3.\n That was your first attempt."
				if data == b'4':
					while data == b'4' :
						#msg = "Well done!"
						#print (msg)	
						conn.sendall(msg)
			    			data_2 = conn.recv(1024)
						if ( data_2 == b'-1'):
							break
					print (msg)
					data_2 = data_2.strip()
					data_2 = int (data_2)
					pub.publish(data_2)
					rospy.loginfo (data_2)
				else:
					conn.sendall(b'0')
			


			    '''
			    while not rospy.is_shutdown():
				ex_id = data #TODO change to 1, 2, or 3
				pub.publish(ex_id)
				rospy.loginfo (ex_id)
				rate.sleep()
			    '''
	def callback ( self, data):
		self.feedback_msg = data.data


if __name__ == '__main__':
    sn = selection_node ()
    rospy.Subscriber('feedback', String, sn.callback)
    sn.talker()



