#! /usr/bin/env python3
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
import math
import rospy

def quat_to_deg(x, y, z, w):
    a = 2.0 * (w * z + x * y)
    b = 1.0 -2.0 * (y * y + z * z)
    yaw = math.atan2(a, b)
    return yaw

def get_steering_angle():
	try:
		
		link_state_get = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		link_state = link_state_get("qcar::hubfr", "base_footprint")
		current_steering_angle = (quat_to_deg(link_state.link_state.pose.orientation.x,link_state.link_state.pose.orientation.y,link_state.link_state.pose.orientation.z,link_state.link_state.pose.orientation.w))
		return current_steering_angle

	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))


def get_qcar_state():
    try:
    	
    	qcar_state_get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    	qcar_state = qcar_state_get("qcar", "")

    	current_x = qcar_state.pose.position.x
    	current_y = qcar_state.pose.position.y
    	current_yaw = quat_to_deg(qcar_state.pose.orientation.x, qcar_state.pose.orientation.y, qcar_state.pose.orientation.z, qcar_state.pose.orientation.w)
    	current_vx = qcar_state.twist.linear.x
    	current_vy = qcar_state.twist.linear.y

    	return current_x, current_y, current_yaw, current_vx, current_vy

    except rospy.ServiceException as e:
    	rospy.loginfo("Get Model State service call failed:  {0}".format(e))

def update_state():

	current_x, current_y, current_yaw, current_vx, current_vy = get_qcar_state()
	current_steering_angle = get_steering_angle()

	return current_x, current_y, current_yaw, current_vx, current_vy, current_steering_angle

if __name__ == '__main__':
	current_x, current_y, current_yaw, current_vx, current_vy, current_steering_angle = update_state()
	print("QCar x:               ", current_x, "\nQCar y:               ", current_y, "\nQCar Yaw:             ", current_yaw, "\nQCar Velocity x:      ", current_vx, "\nQCar Velocity y:      ", current_vy, "\nQCar Steering Angle:  ", current_steering_angle)
