#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

# import rospy which is the python package that deals with ros commands, eg Subscribers, publishers and ros rates
# import message types so that they can be directly accessed rather than typing std_msgs.Float64


class SubscriberClass(object):
	def __init__(self):
		super().__init__() # initalises the class using super instead of class name
		self.subscriber() # intialises the subscriber for the required topic
		self.index_variable = 0.0 # variable that will be grabbed from subscriber 

	def subscriber(self):
		self.index_sub = rospy.Subscriber('random_topic/random_subtopic', Float64, self.index_callback, queue_size=1, buff_size=2**24)
		# Subscriber for set topic, topic name and data type are created in the publisher
		# Subscribers have callbacks that activate everytime the message is published

	def index_callback(self, value):
		self.index_variable = value.data
		# callback saves the published data to a class variable

	def get_index(self):
		return self.index_variable
		# function for returning the class variable


if __name__ == '__main__':
	rospy.init_node('sub_and_pub_demo_node', disable_signals=True)
	#intialises the ros node

	topic_publisher = rospy.Publisher('random_topic/random_subtopic', Float64, queue_size = 0)
	# sets up the publisher, the topic name used can be any name you decide
	# message type can be one of the ros standard messages or you can create custom messages using online tutorials

	index = 0
	A = SubscriberClass()

	rate = rospy.Rate(1)
	#rate.sleep is used to sleep a node for a brief period of time, the 1 sets the frequency of the node
	while not rospy.is_shutdown():
		topic_publisher.publish(index)
		#publish index to the set topic
		index += 1
		print(A.get_index())
		#grab the index data from the subscriber class 

		rate.sleep()
		#pauses the node




