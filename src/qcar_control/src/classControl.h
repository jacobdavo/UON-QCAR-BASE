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
#include "std_msgs/Float32.h"
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
        double getVel();
        int lengthWP();
        int getIndex(float);


        float velocityPID(float velDesired, float vel);
        void command(float omega, float delta);
		
	private:

        ros::NodeHandle* n;
        ros::Subscriber subGuid;
        ros::Subscriber subNav;
        ros::Publisher pubCmdRl;
        ros::Publisher pubCmdRr;
        ros::Publisher pubCmdFl;
        ros::Publisher pubCmdFr;
        ros::Publisher pubCmdFls;
        ros::Publisher pubCmdFrs;

        std_msgs::Float32 velCmdR;
        std_msgs::Float32 velCmdL;
        std_msgs::Float64 angCmd;

        void init_guidSub();
        void guidCallback(const qcar_control::TrajectoryMessage::ConstPtr&);

        void init_navSub();
        void navCallback(const nav_msgs::Odometry::ConstPtr& msg);
        float quat_to_rad(float, float, float, float);

        void init_cmdPub();


        std::vector<double> waypoint_times;
        std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
        double velocity;

        struct States qcarStates;

        float dt = 0;
        float iPrev = 0;
        float ePrev = 0;
        float Kp = 0.8;
        float Ki = 0.5;
        float Kd = 0;
        float r = 0.033;

};

#endif