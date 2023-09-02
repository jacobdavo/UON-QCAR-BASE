#include "ros/ros.h"
#include <stdio.h>
#include <vector>
#include <math.h>

#include "classControl.h"

# define M_PI 3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{
	printf("controller launched...");

	ros::init(argc, argv, "Control_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20);
	printf("OK\n");

    float dt = 0.05;

    classControl qcarController(&n, (float)dt*1e-2);

    while(ros::ok())
    {
        for(int i = 1; i< 100; i++)
        {
            std::cout << qcarController.getWPT(i) << std::endl;
            std::cout << qcarController.getWPY(i) << std::endl;
            std::cout << qcarController.getWPX(i) << std::endl << std::endl;
            ros::spinOnce();
            loop_rate.sleep();
        }   

        return 0;
        
    }   

    return 0;
}