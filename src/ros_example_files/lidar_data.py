#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

class LidarData(object):
	def __init__(self):
		super().__init__()
		self.subscriber()
		self.angle_min = 0.0
		self.angle_max = 0.0
		self.range_min = 0.0
		self.range_max = 0.0
		self.ranges = []
		self.angle_increment = 0.0

	def subscriber(self):
		self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1, buff_size=2**24)

	def lidar_callback(self, value):
		self.ranges = value.ranges
		self.angle_min = value.angle_min
		self.angle_max = value.angle_min
		self.range_min = value.range_min
		self.range_max = value.range_max
		self.angle_increment = value.angle_increment

	def get_angle_data(self):
		return self.angle_min, self.angle_max, self.angle_increment

	def get_ranges(self):
		return self.ranges