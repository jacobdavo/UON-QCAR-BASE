#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

# import rospy which is the python package that deals with ros commands, eg Subscribers, publishers and ros rates
# import message types so that they can be directly accessed rather than typing std_msgs.Float64

if __name__ == '__main__':
	rospy.init_node('publisher_demo_node')
	#intialises the ros node

	topic_publisher = rospy.Publisher('random_topic/random_subtopic', Float64, queue_size = 0)
	# sets up the publisher, the topic name used can be any name you decide
	# message type can be one of the ros standard messages or you can create custom messages using online tutorials

	index = 0

	rate = rospy.Rate(1)
	#rate.sleep is used to sleep a node for a brief period of time, the 1 sets the frequency of the node
	while not rospy.is_shutdown():
		topic_publisher.publish(index)
		#publish index to the set topic
		index += 1

		rate.sleep()
		#pauses the node