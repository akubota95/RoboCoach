#!/usr/bin/env python
"""
Filename: interacting.py
Authors: Alyssa Kubota, Yi Peng, Maryam Pourebadi
Description: Handles RoboCoach interaction with the user.
    Written as part of CSE 276D Healthcare Robotics final project.
"""

import rospy
from numpy import random
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class Interacting():
	def __init__(self):
		# ROS stuff
		self.pub = rospy.Publisher('interact', String, queue_size=10)
		self.nav_sub = rospy.Subscriber('navigate', String, self.nav_callback)
		self.sc = SoundClient()
		# Mapping of interactions to their functions
		self.interactions_dict = {
		    "JOKE"      : self.joke,
		    "ASK"       : self.ask,
		    "GOAL"      : self.goal,
		    "PROUD"     : self.proud,
		    "ENCOURAGE" : self.encourage,
		    "TIP"       : self.tip,
		    "WATER"     : self.water
		}
		self.interactions_list = self.interactions_dict.keys()

	# Perform an interaction
	def do_interaction(self):
		rospy.loginfo("interacting")
		self.interaction()()
		rospy.sleep(10)
		# Tell the navigation node that the interaction is done
		self.pub.publish('Done')

	# Select an interaction to perform
	def interaction(self):
		# For now, randomly select an interaction
		interact_key = random.choice(self.interactions_list)
		return self.interactions_dict.get(interact_key, lambda: "Invalid interaction")

	# Tell a joke to the user
	def joke(self):
		rospy.loginfo("Telling a joke")
		self.sc.say('I have a joke for you.')
		rospy.sleep(4)
		self.sc.say('Why did the robot go back to school?')
		rospy.sleep(4)
		self.sc.say('Because he was getting a little rusty')
		rospy.sleep(4)

	# Ask the user how their day was
	def ask(self):
		rospy.loginfo("Asking about day")
		self.sc.say('Can you tell me about your day?')
		rospy.sleep(45)

	# Ask the user about a goal they have
	def goal(self):
		rospy.loginfo("Asking about goal")
		self.sc.say('What is a goal that you have?')
		rospy.sleep(4)
		self.sc.say('It can be related to anything, such as health, family, or your personal life.')
		rospy.sleep(45)
	
	# Ask the user about something that they are proud about accomplishing last week
	def proud(self):
		rospy.loginfo("Asking about accomplishment")
		self.sc.say('Is there anything you did last week that you are proud of?')
		rospy.sleep(7)
		self.sc.say('Even the smallest accomplishments are worth celebrating!')
		rospy.sleep(45)

	# Encourage the user
	def encourage(self):
		rospy.loginfo("Encouraging")
		self.sc.say("You're doing great!")
		rospy.sleep(3)
		self.sc.say("We're almost at the next stop")
		rospy.sleep(4)

	# Give the user a health tip
	def tip(self):
		rospy.loginfo("Giving a tip")
		self.sc.say("Here's a health tip")
		rospy.sleep(3)
		self.sc.say('Regular exercise can help improve energy levels and memory.')
		rospy.sleep(7)
		self.sc.say('It can also help alleviate depression')
		rospy.sleep(4)
		self.sc.say("It's great that you're walking with me!")
		rospy.sleep(5)

	# Remind the user to drink water
	def water(self):
		rospy.loginfo("Water reminder")
		self.sc.say("Don't forget to stay hydrated!")
		rospy.sleep(4)
		self.sc.say("Remember to drink some water when we finish our walk")
		rospy.sleep(7)
	
	# Begin the interaction when we receive a message from navigation node
	def nav_callback(self, notif):
		msg = notif.data
		rospy.loginfo('Message from nav: ' + msg)
		if msg == "Navigating":
			rospy.sleep(5)
			self.do_interaction()

if __name__ == '__main__':
	# Initialize ROS stuff
	rospy.init_node('interacting', anonymous=False)
	interact = Interacting()
	rospy.loginfo("Waiting to begin interaction")
	rospy.spin()

