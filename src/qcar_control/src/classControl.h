#ifndef CLASS_CONTROL_H
#define CLASS_CONTROL_H

// classControl.h
// control class for ROS control_package

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
        classControl(ros::NodeHandle*);

        States* getStates();
        double getWPX(int);
        double getWPY(int);
        double getWPT(int);
        double getVel();
        std::vector<double>& getWPVec();
        int getIndex(float);
        void command(float, float);
		
	private:

        // private member variables
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

        std::vector<double> waypoint_times;
        std::vector<double> waypoint_x;
        std::vector<double> waypoint_y;
        double velocity;

        struct States qcarStates;

        // private functions
        void init_guidSub();
        void guidCallback(const qcar_control::TrajectoryMessage::ConstPtr&);
        void init_navSub();
        void navCallback(const nav_msgs::Odometry::ConstPtr& msg);
        float quat_to_rad(float, float, float, float);
        void init_cmdPub();

};

#endif