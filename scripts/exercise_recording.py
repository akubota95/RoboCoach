#!/usr/bin/env python

"""
Filename:    exercise_recording.py
Authors:     Maryam Pourebadi, Alyssa Kubota, Yi Peng
Description: Handles the Interactive Ecercise module of the RoboCoach
	     to collect the localization data of the tracked objects while user is performing the excercise,
	     create separate recording lists to store these data for each specifically selected exercise,
	     calculate the number of the correct repetitions for the user's performance,
	     analyze the recording list for the current exercise,
	     generate and publish two feedback messages: 
	     one explaining the correctness of the performance, 
	     and another one explaining the progress of the user.
	     Written as part of CSE 276D Healthcare Robotics final project: RoboCoach.
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from std_msgs.msg import Int16
import numpy

class recording_node(object):
        def __init__(self):
    	    # Params
	    #ID corresponding to the currently active exercise
            self.current_exercise_id = -1
	    #Active exercise flag 
	    self.flag = -1
	    #Lists to store x and y coordinates of pink and green objects
	    self.record_green_x = []
	    self.record_green_y = []
	    self.record_pink_x = []
	    self.record_pink_y = []
	    self.repetition = 0 
	    #To keep record of progress in intensity gradually as tolerated for each excercise
	    self.repetition_green_1 = []
	    self.repetition_pink_1 = []
	    self.repetition_green_2 = []
	    self.repetition_pink_2 = []
	    self.repetition_green_3 = []
	    self.repetition_pink_3 = []
	    #Feedback messages
	    self.msg1 = ""
	    self.msg2 = ""

            # Node cycle rate (in Hz).
            self.loop_rate = rospy.Rate(10)

            #publisher
	    self.pub=rospy.Publisher('feedback', String ,queue_size=1)


	def callback(self, data):
	    
	    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", data)
	    self.current_exercise_id = int (data.data)
	    print (self.current_exercise_id)
	    '''
	    #detect the currently active excercise 
	    try:
		    self.current_exercise_id = data.data.strip()
		    self.current_exercise_id = int (self.current_exercise_id)
	    	    print (self.current_exercise_id)
	    except Exception as e:
		    #rospy.logininfo ('Error: ' + str (e) ) 
	    '''

	def recording(self, data):
	    if (self.current_exercise_id != -1 and self.current_exercise_id  != 4):		
			#Set the current flag to know for which excercise it is recording the movements of the colored objects
			self.flag = self.current_exercise_id

			#Record x and y coordinates for the green/right hand object
			if ( data.data[0] == 11.0 ):
				self.record_green_x.append(data.data[1])
				self.record_green_y.append(data.data[2])
				print ("green")
				print (self.record_green_x) 
				print (self.record_green_y ) 

			#Record x and y coordinates for the pink/left hand object
			elif ( data.data[0] == 22.0 ):
				self.record_pink_x.append(data.data[1])
				self.record_pink_y.append(data.data[2])
				print ("pink")
				print (self.record_pink_x) 
				print (self.record_pink_y ) 

	    #The user has done the selected excercise. 
	    #System will Calculate the number of repetitions for that excercise, publishes the feedbacks and then reset eveyrthing
	    if(self.current_exercise_id == 4):
			
		if ( self.flag == 1 ):  
			#calcualte repetition for green/right and pink/left while user doing excersice #1.
			self.repetition_green_1.append ( self.detect_repetitions(self.record_green_y, threshold1=300, threshold2 = 250, mpd=7))
			self.repetition_pink_1.append (self.detect_repetitions(self.record_pink_y,  threshold1=300, threshold2 = 250, mpd=7))

			#Generate a feedback message based on recorded data
			if ( len(self.repetition_green_1) > 0 and len(self.repetition_pink_1) > 0 ):
				self.msg1 = self.correctness_feedback ( self.repetition_green_1 , self.repetition_pink_1 )
			if ( len(self.repetition_green_1) > 1):
				self.msg2 = self.progress_feedback (self.repetition_green_1 , self.repetition_pink_1)	
			print ( " 1 is selected and ...")
			#reset the flag
			self.flag = 4

		elif ( self.flag == 2 ): 
			#calcualte repetition for green/right and pink/left while user doing excersice #2. 
			self.repetition_green_2.append ( self.detect_repetitions(self.record_green_y, threshold1=150, threshold2 = 120, mpd=10))
			self.repetition_pink_2.append (self.detect_repetitions(self.record_pink_y,  threshold1=140, threshold2 = 110, mpd=10))

			#Generate a feedback message based on recorded data
			if ( len(self.repetition_green_2) > 0 and len(self.repetition_pink_2) > 0 ):
				self.msg1 = self.correctness_feedback ( self.repetition_green_2 , self.repetition_pink_2 )
			if ( len(self.repetition_green_2) > 1):
				self.msg2 = self.progress_feedback (self.repetition_green_2 , self.repetition_pink_2)	
			#reset the flag
			self.flag = 4

		elif ( self.flag == 3 ): 
			#calcualte repetition for green/right and pink/left while user doing excersice #3.
			self.repetition_green_3.append ( self.detect_repetitions(self.record_green_y, threshold1=150, threshold2 = 120, mpd=10))
			self.repetition_pink_3.append (self.detect_repetitions(self.record_pink_y,  threshold1=140, threshold2 = 110, mpd=10))

			#Generate a feedback message based on recorded data
			if ( len(self.repetition_green_2) > 0 and len(self.repetition_pink_2) > 0 ):
				self.msg1 = self.correctness_feedback ( self.repetition_green_2 , self.repetition_pink_2 )
				self.msg2 = self.progress_feedback (self.repetition_green_2 , self.repetition_pink_2)	
			#reset the flag
			self.flag = 4
		
		
		if ( self.flag == 4 ):

			#Publish two feedbacks
			self.pub.publish(self.msg1) 
			print ( self.msg1 )
 			self.pub.publish(self.msg2) 
			print ( self.msg2 ) 

			#reset lists of x and y coordinations
			self.record_green_x = []
			self.record_green_y = []


	#To calcualte the number of repetition peaks using two thresholds 
	def detect_repetitions(self, coordinate_values, threshold1, threshold2, mpd):
		self.repetition = 0
		ind = 0
		while (ind < len(coordinate_values)-mpd-1):
			if (coordinate_values[ind] > threshold1 and coordinate_values[ind+mpd] < threshold2):
				self.repetition += 1
				ind += mpd
			else:
				ind += 1
		return self.repetition
				
	#Function to check the correctness of the performance based on the repetitions for the left and right hands and accordingly give a feedback to the user
	def correctness_feedback (self, repetition_green, repetition_pink ):

		#print ( "The number of repetition for green :  " , repetition_green )
		#print ( "The number of repetition for pink :  " , repetition_pink  )

		#Give the user feedback based on his/her performance and the number of repetitions on each side
		if ( repetition_green[len(repetition_green)-1]  < (repetition_pink[len(repetition_pink)-1] +1) and repetition_green[len(repetition_green)-1]  > (repetition_pink[len(repetition_pink)-1] -1)  ):
			msg = "Well done with Exercise " + str (self.flag) + ". \n"
		elif ( repetition_green[len(repetition_green)-1]  > (repetition_pink[len(repetition_pink)-1] +1) ):
			msg = "Move your right arm more during Exercise " + str (self.flag) + ". \n"
		elif (repetition_green[len(repetition_green)-1]  < (repetition_pink[len(repetition_pink)-1] -1) ):  
			msg = "Move your left arm more during Exercise " + str (self.flag) + ". \n"
		return msg


	#Funtion to compare the progress while doing an excersice for the second time and give a feedback to the user accordingly 
	def progress_feedback (self, repetition_green , repetition_pink ):
		if len(self.repetition_green_2) == 1 :
			msg = "That was your first attempt."
		else:
			dif = repetition_green[len(repetition_green)-2] -  repetition_green[len(repetition_green)-1]
			if (dif == 0 and repetition_green[len(repetition_green)-2] == 0):
				msg = "Lets do some workout together!"
			elif (dif == 0 ):
				msg = "You are performing as good as before; keep it up!"
			elif ( dif > 0):
				msg= "Your performance has improved!" 
			elif ( dif < 0):
				msg = "Your records show that you can do better!"
		return msg
	

	def start(self):

	    #gets the current status of the excerxises: exercises = [0, 0, 0], 0: active, 1:deactive
	    rospy.Subscriber('exercise', Int16, self.callback)
	    #rospy.Subscriber('socket', String, self.callback)
	   

	    #gets x and y coordinates of the green object and pink object
	    rospy.Subscriber('coordinates', numpy_msg(Floats), self.recording)

	    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('exercise_recording', anonymous=True)
    my_node = recording_node()
    my_node.start()



