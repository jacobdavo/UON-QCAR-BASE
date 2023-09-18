#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class SubscriberClass(object):
	def __init__(self):
		super().__init__() # initalises the class using super instead of class name
		self.subscriber() # intialises the subscriber for the required topic
		self.index_variable = 0.0 # variable that will be grabbed from publisher

	def subscriber(self):
		self.index_sub = rospy.Subscriber('random_topic/random_subtopic', Float64, self.index_callback)
		# Subscriber for set topic, topic name and data type are created in the publisher
		# Subscribers have callbacks that activate everytime the message is published

	def index_callback(self, value):
		self.index_variable = value.data
		print("Published Index:", self.index_variable)
		# callback saves the published data to a class variable

if __name__ == '__main__':
	rospy.init_node('subscriber_demo_node')
	#intialises the ros node

	SubscriberClass()
	#initalises the class

	rospy.spin()
	# keeps python from exiting until node is closed
