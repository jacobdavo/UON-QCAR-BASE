#include "ros/ros.h"
#include "classNav.h"


classNav::classNav(ros::NodeHandle* _n)
{
    n = _n;
    init_imuSub();
    init_rlEncoderSub();
}

void classNav::init_imuSub()
{
    imuSub = n->subscribe("/imu", 0, &classNav::imuCallback, this);
}

void classNav::init_rlEncoderSub()
{
    rlEncoderSub = n->subscribe("/wheelrl_motor/encoder", 0, &classNav::rlEncoderCallback, this);
}

void classNav::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    angZ = quatToRad(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w); 
    velZ = msg->angular_velocity.z;
    accZ = msg->linear_acceleration.z;
}

void classNav::rlEncoderCallback(const std_msgs::Int32::ConstPtr& msg)
{
    rlEncoder = msg->data;
}

double classNav::getAccZ()
{
    return accZ;
}

double classNav::getvelZ()
{
    return velZ;
}

double classNav::getangZ()
{
    return angZ;
}

int classNav::getEncoderRL()
{
    return rlEncoder;
}

float classNav::quatToRad(float x, float y, float z, float w)
{
	float yaw;
	float a;
	float b;

	a = 2.0 * (w * z + x * y);
    b = 1.0 -2.0 * (y * y + z * z);
    yaw = atan2(a, b);

    return yaw;
}