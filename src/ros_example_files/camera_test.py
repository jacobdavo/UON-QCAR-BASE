#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



def depth_camera_callback(data):
	try:
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

		min_depth = 0
		max_depth = 10
		cv_image = np.clip(cv_image, min_depth, max_depth)
		cv_image = (cv_image - min_depth)/(max_depth - min_depth)
		cv_image = (cv_image * 255).astype(np.uint8)

		cv2.imshow("Depth Camera Image", cv_image)
		cv2.waitKey(1)

	except Exception as e:
		rospy.logerr("Error Message: %s" , str(e))

def camera_callback(data):
	try:
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

		cv2.imshow("Camera Image", cv_image)
		cv2.waitKey(1)

	except Exception as e:
		rospy.logerr("Error Message: %s" , str(e))

def main():
	rospy.init_node("camera_viewer", anonymous=True)
	depth_camera_topic = "/intel_depth_camera/depth/image_raw"
	rospy.Subscriber(depth_camera_topic, Image, depth_camera_callback)

	camera_topic = "/front_camera/image_raw"
	rospy.Subscriber(camera_topic, Image,camera_callback)


	rospy.spin()

if __name__ == "__main__":
	main()