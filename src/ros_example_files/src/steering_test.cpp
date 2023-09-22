#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "stdio.h"
#include "math.h"

# define M_PI 3.14159265358979323846 /* pi */ 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "steering_test_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20); // set loop rate 20Hz

    // Define publishers
    ros::Publisher pubAngL = n.advertise<std_msgs::Float64>("qcar/base_fl_controller/command", 1);
    ros::Publisher pubAngR = n.advertise<std_msgs::Float64>("qcar/base_fr_controller/command", 1);

    // create message object
    std_msgs::Float64 angCmd;

    for(float angle = -30; angle <= 30; angle++)
    {
        // populate message with data and publish to topic
        angCmd.data = angle*M_PI/180.0;
        pubAngL.publish(angCmd);
        pubAngR.publish(angCmd);

        // spin ROS
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}