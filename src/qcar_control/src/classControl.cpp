// classControl.cpp
// control class for ROS control_package
// Elliott G. Shore
// University of Newcastle, July 2023

#include "ros/ros.h"
#include "classControl.h"


classControl::classControl(ros::NodeHandle* _n, float dt_)
{
    n = _n;
	dt = dt_;
    init_guidSub();
    init_navSub();
    init_cmdPub();
    printf("classControl successfuly created!\n");
};

void classControl::init_guidSub()
{
    subGuid = n->subscribe("/qcar/trajectory_topic", 0, &classControl::guidCallback, this);
}

void classControl::guidCallback(const qcar_control::TrajectoryMessage::ConstPtr& msg)
{
	waypoint_times = msg->waypoint_times;
    //std::cout << waypoint_times.front() << std::endl;
    waypoint_x = msg->waypoint_x;
    waypoint_y = msg->waypoint_y;
    //printf("Test\n");
}

double classControl::getWPX(int idx)
{
    if(!waypoint_x.empty())
    {
        return waypoint_x.at(idx);
    }

    return -1; // no waypoints
}

double classControl::getWPY(int idx)
{
    if(!waypoint_y.empty())
    {
        return waypoint_y.at(idx);
    }

    return -1; // no waypoints
}

double classControl::getWPT(int idx)
{
    if(!waypoint_times.empty())
    {
        return waypoint_times.at(idx);
    }

    return -1; // no waypoints
}

// this callback should talk to the navigation topics i.e not /odom. Talk to the simulated sensors instead.
void classControl::init_navSub()
{
	subNav = n->subscribe("/odom", 0, &classControl::navCallback, this);
}

void classControl::navCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	qcarStates.North = msg->pose.pose.position.y;
	qcarStates.East = msg->pose.pose.position.x;
	qcarStates.Psi = quat_to_rad(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	qcarStates.Vel = sqrt(pow(msg->twist.twist.linear.x,2) + pow(msg->twist.twist.linear.y,2));
    //printf("test2\n");
}

States* classControl::getStates()
{
	return &qcarStates;
}

float classControl::quat_to_rad(float x, float y, float z, float w)
{
	float yaw;
	float a;
	float b;

	a = 2.0 * (w * z + x * y);
    b = 1.0 -2.0 * (y * y + z * z);
    yaw = atan2(a, b);

    return yaw;
}

int classControl::lengthWP()
{
    return waypoint_times.size();
}

void classControl::init_cmdPub()
{

	pubCmdRl = n->advertise<std_msgs::Float32>("/wheelrl_motor/command", 1);
    pubCmdRr = n->advertise<std_msgs::Float32>("wheelrr_motor/command", 1);
    pubCmdFl = n->advertise<std_msgs::Float32>("/wheelfl_motor/command", 1);
    pubCmdFr = n->advertise<std_msgs::Float32>("wheelfr_motor/command", 1);

    pubCmdFls = n->advertise<std_msgs::Float64>("qcar/base_fl_controller/command", 1);
    pubCmdFrs = n->advertise<std_msgs::Float64>("qcar/base_fr_controller/command", 1);
}

void classControl::command(float omega, float delta)
{

	velCmdL.data = -omega;
	velCmdR.data = omega;
	angCmd.data = delta;

	pubCmdRl.publish(velCmdL);
    pubCmdRr.publish(velCmdR);
    pubCmdFl.publish(velCmdL);
    pubCmdFr.publish(velCmdR);

    pubCmdFls.publish(angCmd);
    pubCmdFrs.publish(angCmd);
}

float classControl::velocityPID(float velDesired, float vel)
{
	float P;
	float I;
	float D;
	float e;
	float u;

	e = velDesired - vel;

	P = e;
	I = iPrev + e*dt;
	D = (e - ePrev)/dt;

	u = (Kp*P + Ki*I + Kd*D)/r;

	iPrev = I;
	ePrev = e;

	return u;
}