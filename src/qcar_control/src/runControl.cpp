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
	ros::Rate loop_rate(20); // 20Hz
	printf("OK\n");

    float dt = 5; // dt*1e2
    float omega = 0;
    float delta = 0;
    float desHead = 0;
    float uHead = 0;
    float desVel = 0;
    float uVel = 0;
    float time = 0;
    int indexCurrent = 0;

    classControl qcarController(&n, (float)dt*1e-2);

    while(ros::ok())
    {
        if(qcarController.getWPT(0) != -1)
        {
            for(int i = 1; i < qcarController.lengthWP()*(1/(dt*1e-2)); i++)
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

                std::cout << "[Time: " << time << "]" << "desHead: " << desHead*180/M_PI << "desVel: " << desVel << std::endl;
                std::cout << "[Time: " << time << "]" << "actHead: " << qcarController.getStates()->Psi*180/M_PI << "actVel: " << qcarController.getStates()->Vel << std::endl;

                time = time + dt;

                ros::spinOnce();
                loop_rate.sleep();
            }

        }

        qcarController.command(0, 0);

        if(time > 1)
        {
            return 0;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }   

    return 0;
}