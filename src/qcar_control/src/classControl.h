#ifndef CLASS_CONTROL_H
#define CLASS_CONTROL_H

// classControl.h
// control class for ROS control_package
// Elliott G. Shore
// University of Newcastle, July 2023

#include "ros/ros.h"
#include <vector>
#include <stdio.h>
#include "qcar_control/TrajectoryMessage.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"

struct States
{
        float North;
        float East;
        float Psi;
        float Vel;
};

class classControl
{
	public:

        // public functions
        classControl(ros::NodeHandle*, float);

        States* getStates();
        double getWPX(int);
        double getWPY(int);
        double getWPT(int);
		
	private:

        ros::NodeHandle* n;
        ros::Subscriber subGuid;
        ros::Subscriber subNav;

        void init_guidSub();
        void guidCallback(const qcar_control::TrajectoryMessage::ConstPtr&);

        void init_navSub();
        void navCallback(const nav_msgs::Odometry::ConstPtr& msg);
        float quat_to_rad(float, float, float, float);


        std::vector<double> waypoint_times;
        std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;

        struct States qcarStates;

        float dt = 0;

};

#endif