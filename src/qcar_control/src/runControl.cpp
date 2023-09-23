#include "ros/ros.h"
#include <stdio.h>
#include <vector>
#include <math.h>

#include "classControl.h"

# define M_PI 3.14159265358979323846  // pi

int main(int argc, char **argv)
{
	printf("[Control_Node] controller launched...");

	ros::init(argc, argv, "Control_node");
    ros::NodeHandle n;
	ros::Rate loop_rate(20); // 20Hz
    printf("OK\n");

    float dt = 5; // dt*1e-2
    float omega = 0;
    float delta = 0;
    float desHead = 0;
    float uHead = 0;
    float time = 0;
    int indexCurrent = 0;

    bool waiting = true;

    classControl qcarController(&n);

    while(ros::ok())
    {
        if(waiting){
            std::cout<<"[Control_Node] Waiting for trajectory..." << std::flush;
            waiting = false;
        }
        
        if(qcarController.getWPT(0) != -1)
        {
            printf("[Control_Node] Trajectory received...STARTING\n\n");
            while(ros::ok())
            {

                indexCurrent = qcarController.getIndex((float)time*1e-2);

                desHead = atan2(qcarController.getWPY(indexCurrent + 1) - qcarController.getWPY(indexCurrent), qcarController.getWPX(indexCurrent + 1) - qcarController.getWPX(indexCurrent));

                uHead = atan((2*0.3*sin(desHead - qcarController.getStates()->Psi))/(0.26));

                if(uHead*180/M_PI > 30)
				{
					uHead = 30*M_PI/180;
				}
				else if(uHead*180/M_PI < -30)
				{
					uHead = -30*M_PI/180;
				}

				delta = uHead;

                omega = qcarController.getVel()/0.033;

                qcarController.command(omega, delta);

                time = time + dt;

                ros::spinOnce();
                loop_rate.sleep();

                std::cout << "[Control_Node]" << std::endl;
                std::cout << "[time]: " << std::fixed << std::setprecision(2) << time*1e-2 << " secs\n";
                std::cout << "[omga]: " << std::fixed << std::setprecision(2) << omega << " rad/s\n";
                std::cout << "[dlta]: " << std::fixed << std::setprecision(2) << delta*180.0/M_PI << " deg\n\n\n";

                if(time*1e-2 > qcarController.getWPVec().back()-1)
                {
                    qcarController.command(0, 0);
                    ros::spinOnce();
                    loop_rate.sleep();
                    return EXIT_SUCCESS;
                }

            }

            qcarController.command(0, 0);
            return EXIT_FAILURE;

        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return EXIT_FAILURE;
    std::cout << "ROS NOT OK :()...\n";
}