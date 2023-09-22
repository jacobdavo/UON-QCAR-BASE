#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "stdio.h"

// callback function
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::cout << "angular velocity in z: " << msg->angular_velocity.z << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_test_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20); // set loop rate 20Hz

    ros::Subscriber imuSub = n.subscribe("/imu", 0, imuCallback); // Define subcriber

    ros::spin(); // spin ROS

    return 0;
}