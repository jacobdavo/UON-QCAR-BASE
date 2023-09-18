// classControl.cpp
// control class for ROS control_package

#include "ros/ros.h"
#include "classControl.h"

// Constructor for class classControl
classControl::classControl(ros::NodeHandle* _n)
{
    n = _n;
    init_guidSub();
    init_navSub();
    init_cmdPub();
    printf("classControl successfuly created!\n");
};

// guidance subscriber intialiser
void classControl::init_guidSub()
{
    subGuid = n->subscribe("/qcar/trajectory_topic", 0, &classControl::guidCallback, this);
}

// guidance subscriber callback function
void classControl::guidCallback(const qcar_control::TrajectoryMessage::ConstPtr& msg)
{
	waypoint_times = msg->waypoint_times;
    waypoint_x = msg->waypoint_x;
    waypoint_y = msg->waypoint_y;
    velocity = msg->velocity;
}

// get x-coord. of waypoints
double classControl::getWPX(int idx)
{
    if(!waypoint_x.empty())
    {
        return waypoint_x.at(idx);
    }

    return -1; // no waypoints
}

// get y-coord. of waypoints
double classControl::getWPY(int idx)
{
    if(!waypoint_y.empty())
    {
        return waypoint_y.at(idx);
    }

    return -1; // no waypoints
}

// get time of waypoints
double classControl::getWPT(int idx)
{
    if(!waypoint_times.empty())
    {
        return waypoint_times.at(idx);
    }

    return -1; // no waypoints
}

// get velocity of waypoints
double classControl::getVel()
{
    if(!waypoint_times.empty())
    {
        return velocity;
    }

    return -1; // no waypoints
}

// navigation subscriber initialiser
// this callback should talk to the navigation topics i.e not /odom. Talk to the simulated sensors instead.
void classControl::init_navSub()
{
	subNav = n->subscribe("/odom", 0, &classControl::navCallback, this);
}

// navigation suscriber callback function
void classControl::navCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	qcarStates.North = msg->pose.pose.position.y;
	qcarStates.East = msg->pose.pose.position.x;
	qcarStates.Psi = quat_to_rad(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	qcarStates.Vel = sqrt(pow(msg->twist.twist.linear.x,2) + pow(msg->twist.twist.linear.y,2));
}

// get qcarStates structure
States* classControl::getStates()
{
	return &qcarStates;
}

// quaternions to rads conversion function
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

// get the length of waypoints vector
std::vector<double>& classControl::getWPVec()
{
    return waypoint_times;
}

// command publisher initialiser 
void classControl::init_cmdPub()
{

	pubCmdRl = n->advertise<std_msgs::Float32>("/wheelrl_motor/command", 1);
    pubCmdRr = n->advertise<std_msgs::Float32>("wheelrr_motor/command", 1);
    pubCmdFl = n->advertise<std_msgs::Float32>("/wheelfl_motor/command", 1);
    pubCmdFr = n->advertise<std_msgs::Float32>("wheelfr_motor/command", 1);

    pubCmdFls = n->advertise<std_msgs::Float64>("qcar/base_fl_controller/command", 1);
    pubCmdFrs = n->advertise<std_msgs::Float64>("qcar/base_fr_controller/command", 1);
}

// command publisher function
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

// get the current index of waypoints
int classControl::getIndex(float currentTime)
{
    int i = 0;
    int k = 0;

    while((float)waypoint_times.at(i) <= currentTime)
    {
        i++;
        k = i;
    }

    return k;
}