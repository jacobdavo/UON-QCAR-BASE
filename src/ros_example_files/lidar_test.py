#! /usr/bin/env python3
import rospy
import lidar_data
from time import sleep
import math
import matplotlib.pyplot as plt

if __name__ == '__main__':
	rospy.init_node('lidar_node', disable_signals=True)

	rosrunning = 0
	while rosrunning==0:
		sleep(1)
		if rospy.Time.now() != rospy.Time():
			rosrunning = 1

	rate = rospy.Rate(0.2)
	scan_sub = lidar_data.LidarData() #create lidar data class from lidar_data.py file
	rate.sleep()
	angle_min, angle_max, angle_increment = scan_sub.get_angle_data()
	print(angle_min)
	print(angle_max)
	print(angle_increment)
	print(angle_increment*360)
	print("starting")
		
	ranges = scan_sub.get_ranges()

	plt.figure()
	current_angle = angle_min
	plot_angle = 0.0
	plt.plot(0,0,marker="o",markersize=5,markerfacecolor="black", markeredgecolor="black")

	for i in range(0,len(ranges)):
		if ranges[i] != math.inf:
			x = ranges[i]*math.cos(current_angle+math.pi/2)
			y = ranges[i]*math.sin(current_angle+math.pi/2)
			plt.plot(x,y,marker="o", markersize=5, markerfacecolor="green", markeredgecolor="green")

		current_angle = current_angle + angle_increment

	plt.show()