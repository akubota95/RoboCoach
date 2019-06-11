#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String
from std_msgs.msg import Int16
import numpy


def talker():

    pub=rospy.Publisher('exercises', Int16, queue_size=10)
    rospy.init_node('exercises_selection', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    '''
    for i in range (500):
	ex_id = 1 #TODO change to 1
	pub.publish(ex_id)
	rospy.loginfo (ex_id)
	rate.sleep()
    '''
    while not rospy.is_shutdown():
	ex_id = -1 #TODO change to 1, 2, or 3
	pub.publish(ex_id)
	rospy.loginfo (ex_id)
	rate.sleep()

if __name__ == '__main__':
    talker()
