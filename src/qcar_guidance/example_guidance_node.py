#! /usr/bin/env python3

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from qcar_guidance.msg import TrajectoryMessage
from time import sleep
from scipy.interpolate import interp1d

def midpoint(point1,point2):
	x_midpoint = (point1[0] + point2[0])/2
	y_midpoint = (point1[1] + point2[1])/2
	return x_midpoint, y_midpoint

def get_cone_positions():
	try:
		model_names_get = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
		cone_state_get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		model_names = model_names_get()
		model_names_length = len(model_names.model_names)
		cone_position_array = np.empty([model_names_length, 2])
		blue_cones_x = []
		blue_cones_y = []
		yellow_cones_x = []
		yellow_cones_y = []

		complete = 0
		index = 0
		while complete == 0:
			search_item = "blue_cone_{}".format(index)
			if search_item in model_names.model_names:
				current_cone = cone_state_get(search_item,"")
				blue_cones_x.append(current_cone.pose.position.x)
				blue_cones_y.append(current_cone.pose.position.y)
				search_item	= "yellow_cone_{}".format(index)
				current_cone = cone_state_get(search_item,"")
				yellow_cones_x.append(current_cone.pose.position.x)
				yellow_cones_y.append(current_cone.pose.position.y)
			else:
				complete = 1
			index += 1

		return blue_cones_x,blue_cones_y, yellow_cones_x, yellow_cones_y

	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))

if __name__ == '__main__':
	rospy.init_node('example_guidance_node', disable_signals=True)


	# prevents the node from running whilst ros/gazebo is starting up
	rosrunning = 0
	while rosrunning==0:
		sleep(1)
		if rospy.Time.now() != rospy.Time():
			rosrunning = 1

	trajectory_publisher = rospy.Publisher('/qcar/trajectory_topic', TrajectoryMessage, queue_size = 0)

	blue_cones_x,blue_cones_y, yellow_cones_x, yellow_cones_y = get_cone_positions()
	midpoints = np.empty([2,len(blue_cones_x)+1])
	waypoint_times = [0]
	velocity = 0.5

	for i in range(0,len(blue_cones_x)):
		x_midpoint, y_midpoint = midpoint([blue_cones_x[i],blue_cones_y[i]], [yellow_cones_x[i],yellow_cones_y[i]])
		midpoints[0,i] = x_midpoint
		midpoints[1,i] = y_midpoint
		if i != 0:
			waypoint_distance = math.dist([midpoints[0,i-1],midpoints[1,i-1]],[midpoints[0,i],midpoints[1,i]])
			waypoint_times.append(waypoint_times[-1] + waypoint_distance/velocity)

	midpoints[0,-1] = midpoints[0,0]
	midpoints[1,-1] = midpoints[1,0]
	blue_cones_x.append(blue_cones_x[0])
	blue_cones_y.append(blue_cones_y[0])
	yellow_cones_x.append(yellow_cones_x[0])
	yellow_cones_y.append(yellow_cones_y[0])


	midpoints_distance = (np.cumsum( np.sqrt(np.sum( np.diff(midpoints, axis=1)**2, axis=0))))
	midpoints_distance_total = midpoints_distance[-1]
	midpoints_alpha = np.linspace(0,1,int(round(8*midpoints_distance_total)))
	midpoints_distance = np.insert(midpoints_distance, 0, 0)
	midpoints_distance = midpoints_distance/midpoints_distance[-1]
	midpoints_interpolator = interp1d(midpoints_distance, midpoints, kind = 'quadratic', axis = 1)
	midpoints = midpoints_interpolator(midpoints_alpha)

	plt.figure()
	plt.plot(blue_cones_x,blue_cones_y,marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")
	plt.plot(yellow_cones_x,yellow_cones_y,marker="o", markersize=10, markeredgecolor="yellow", markerfacecolor="yellow")
	plt.plot(midpoints[0],midpoints[1],marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red")
	plt.title('Start Track')
	ax = plt.gca()
	ax.set_aspect('equal', adjustable='box')
	plt.show()

	rate = rospy.Rate(0.2)
	while not rospy.is_shutdown():
		trajectoryTopic = TrajectoryMessage()
		trajectoryTopic.waypoint_times = waypoint_times
		trajectoryTopic.waypoint_x = midpoints[0]
		trajectoryTopic.waypoint_y = midpoints[1]
		trajectoryTopic.velocity = velocity
		trajectory_publisher.publish(trajectoryTopic)
		rospy.loginfo("Trajectory Published")
		rate.sleep()

