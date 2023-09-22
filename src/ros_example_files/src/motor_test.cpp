#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "stdio.h"

// callback function
void callbackEncoder(const std_msgs::Int32::ConstPtr& msg)
{
    std::cout << "Encoder count: " << msg->data << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_test_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20); // set loop rate 20Hz

    // Define publisher and subscriber
    ros::Publisher pubCmdRl = n.advertise<std_msgs::Float32>("/wheelrl_motor/command", 1);
    ros::Subscriber subRl = n.subscribe("/wheelrl_motor/encoder", 1, &callbackEncoder);

    float velocity = 15.0; // rad/s

    // create message object and populate
    std_msgs::Float32 velCmdL;
    velCmdL.data = velocity;

    while(ros::ok())
    {
        pubCmdRl.publish(velCmdL); // publish message to topic

        // spin ROS
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}