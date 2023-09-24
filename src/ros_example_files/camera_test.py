#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv_bridge
import cv2

def depth_camera_callback(data):
	try:
		bridge = cv_bridge.CvBridge()
		depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
		cv2.imshow("Depth Image", depth_image)
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
	depth_camera_topic = "/depth_camera/depth/image_raw"
	rospy.Subscriber(depth_camera_topic, Image, depth_camera_callback)
	camera_topic = "/front_camera/image_raw"
	rospy.Subscriber(camera_topic, Image,camera_callback)
	rospy.spin()

if __name__ == "__main__":
	main()