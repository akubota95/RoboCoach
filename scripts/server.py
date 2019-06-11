#!/usr/bin/env python

"""
Filename: server.pyp
Authors: Alyssa Kubota, Yi Peng, Maryam Pourebadi
Description: Opens a server to listen for requests from the user. 
    Used to select which location RoboCoach should move to while walking
    with the user.
    Written as part of CSE 276D Healthcare Robotics final project.
"""

import rospy
import socket
from std_msgs.msg import String
	
class Connection():

	def __init__(self):
		self.host = ''
		self.port = 65430
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.conn = None
		# ROS Stuff
		self.sub = rospy.Subscriber('navigate', String, self.nav_callback)
		self.pub = rospy.Publisher('socket', String, queue_size=10)

	# Wait for a connection from a remote client
	def wait_for_connection(self):
		self.s.bind((self.host, self.port))
		self.s.listen(1)
		rospy.loginfo("Now listening for connections")
		# Waits until connection is established
		self.conn, addr = self.s.accept()
		# Prevent timeouts
		self.conn.setblocking(1)
		rospy.loginfo("Connected")	

	# Receive data from remote client and publish it to ROS topic
	def wait_for_msg(self):
		rospy.loginfo("Waiting to receive message")
		data = self.conn.recv(1024)
		if data:
			self.pub.publish(data)
			print(data)

	# Send a message to the remote client
	def send_msg(self, msg):
		rospy.loginfo('sending message to client')
		self.conn.sendall(b'1')

	# Callback for the navigation node
	def nav_callback(self, message):
		rospy.loginfo("Done moving")
		print(message)
		if message.data == "MoveDone":
			self.send_msg(message)

if __name__ == '__main__':
	# Initialize ROS stuff
	rospy.init_node('socket_connect', anonymous=False)
	# Open the connection
	c = Connection()
	c.wait_for_connection()
	while not rospy.is_shutdown():
		rospy.loginfo("About to wait for message")
		c.wait_for_msg()
	rospy.spin()
