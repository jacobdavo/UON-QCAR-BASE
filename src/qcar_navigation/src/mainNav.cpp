#include "ros/ros.h"
#include <stdio.h>
#include <math.h>

#include "classNav.h"

int main(int argc, char **argv)
{
    printf("Navigation Launched...");

    ros::init(argc, argv, "Navigation_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20);

    printf("OK\n");

    classNav qcarNavigation(&n);

    double angZ;
    double velZ;
    double accZ;

    int rlEncoder;



    while(ros::ok())
    {
        accZ = qcarNavigation.getAccZ();
        rlEncoder = qcarNavigation.getEncoderRL();

        printf("Acc_Z:= %lf\n", accZ);
        printf("Encoder_RL:= %d\n", rlEncoder);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
