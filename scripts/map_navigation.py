#!/usr/bin/env python
"""
Filename: map_navigation.py
Authors: Alyssa Kubota, Yi Peng, Maryam Pourebadi
Description: Handles RoboCoach navigation from one point to another.
    While moving, RoboCoach will interact with a nearby user.
    Written as part of CSE 276D Healthcare Robotics final project.
Notes: Adapted from Gaitech EDU's map_naviation.py
    https://github.com/aniskoubaa/gaitech_edu/blob/master/src/turtlebot/navigation/map_navigation/scripts/map_navigation.py
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs import *
from geometry_msgs.msg import Point
from std_msgs.msg import String

class MapNavigation():
	
	def __init__(self):
		# Initialize coordinates of interest
		self.loc1 = Point(-22.29, 2.58, 0.0)  # Kitchen
		self.loc2 = Point(-15.70, 0.81, 0.0) # Restroom
		self.loc3 = Point(-19.73, 10.26, 0.0) # Lab (lounge)
		self.loc4 = Point(-4.14, -10.65, 0.0) # Desks/home
		self.locs = [self.loc1, self.loc2, self.loc3, self.loc4]
		self.num_locs = len(self.locs)
		# Which location to go to, specified by the remote client
		self.chosen_loc = -1
		# Whether or not we have processed which location to move to
		self.received_loc = False
		# Whether or not the robot has reached the goal
		self.goalReached = False
		# Whether or not the robot is interacting with the person
		self.interacting = False
		# ROS publisher
		self.pub = rospy.Publisher('navigate', String, queue_size=10)
		# ROS subscriber to interaction node	
		rospy.Subscriber('interact', String, self.interact_callback)
		rospy.Subscriber('socket', String, self.socket_callback)

	# Move to the selected location
	def navigate(self):
		destination = self.get_destination()
		# Processed location, so reset received_loc
		self.received_loc = False
		print('Destination chosen')
		self.move_to_goal(destination)

	# Determines which location to move to
	def get_destination(self):
		while not self.received_loc:
			# chosen_loc >= 0 when we've received input from client
			if self.chosen_loc >= 0:
				location = self.locs[self.chosen_loc]
				self.received_loc = True
				self.chosen_loc = -1
		return location

	# Used for testing, probably delete
	def choose_destination(self):
		choice = 'q'

		print(',~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~,')
		print('| Where would you like to go?  |')
		print('| ---------------------------- |')
		print('| 1 : Location 1               |')
		print('| 2 : Location 2               |')
		print('| 3 : Location 3               |')
		print('| ---------------------------- |')
		print('| Press the corresponding key  |')
		print('`~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`')
		
		validChoice = False
		while not validChoice:
			choice = input()
			# Check if input is an integer
			try:
				choice = int(choice)
			except ValueError as e:
				rospy.loginfo('Error: ' + e)
				rospy.loginfo('Please enter an integer')
				continue
			# Offset by 1 to zero index
			choice -= 1
			# Check if input is in the list
			if choice >= self.num_locs:
				rospy.loginfo('Please select a listed location')
				continue
			validChoice = True
		
		location = self.locs[choice]

		return location

	# Move to the specified location
	def move_to_goal(self, location):
		# Define action client to send goal requests to
		ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		# Wait for action server to come up
		while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo('Waiting for action server to come up')

		rospy.loginfo('Action server connected')
		
		# Set goal frame parameters
		goal = MoveBaseGoal()
		## map frame means points will be absolute in the world
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()

		# Move to the goal
		## Target location: (x, y, 0.0)
		goal.target_pose.pose.position = location
		## Target orientation: (0, 0, 0, 1)
		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0

		rospy.loginfo('Sending goal location')
		ac.send_goal(goal, done_cb=self.done_callback)
		rospy.loginfo('hello')

		# goalReached should be True once RoboCoach reaches the goal
		while not self.goalReached:
			# interacting should be False when done interacting
			if not self.interacting:
				self.pub.publish('Navigating')
				self.interacting = True
		rospy.loginfo("Reached goal!")

		# Reached goal so reset goal_reached
		self.goal_reached = False

	# Called when actions transition to Done.
	# Send a message to the client that we reached our goal.
	def done_callback(self, state, result):
		rospy.loginfo(str(state))
		self.goalReached = True
		rospy.loginfo('Done moving')
		self.pub.publish('MoveDone')
	
	# Called when the interaction with the robot is completed
	def interact_callback(self, notif):
		self.interacting = False
		rospy.loginfo('Message from interact: ' + notif.data)

	# Called when we receive data from the remote client.
	# Sets the location to move to.
	def socket_callback(self, received_data):
		loc = received_data.data.strip()
		try:
			self.chosen_loc = int(loc) - 1
			rospy.loginfo('Message from interface: ' + loc)
		except Exception as e:
			rospy.loginfo('Error: ' + str(e))

	# Stop the robot
	def shutdown(self):
		rospy.loginfo("Shutdown command received. Quitting program")
		rospy.sleep()

if __name__ == '__main__':
	try:
		# Initialize ROS stuff
		rospy.init_node('map_navigation', anonymous=False)
		r = rospy.Rate(10)
		# Start the navigation
		mn = MapNavigation()
		while not rospy.is_shutdown():
			rospy.loginfo('Now ready to move')
			mn.navigate()
			r.sleep()
		rospy.spin()

	except rospy.ROSInterruptException as e:
		rospy.loginfo("map_navigation node terminated: " + e)

