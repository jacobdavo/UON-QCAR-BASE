#ifndef CLASS_NAV_H
#define CLASS_NAV_H

// navClass.h
// navigation class for ROS navigation_package
// Elliott G. Shore
// University of Newcastle, July 2023

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"

class classNav
{
	public:

                // class constructor
                classNav(ros::NodeHandle*);

                // getter functions
                double getangZ();
                double getvelZ();
                double getAccZ();
                
                int getEncoderRL();
		

	private:
        
                // initialiser functions functions
                void init_imuSub();
                void init_rlEncoderSub();

                // callback functions
                void imuCallback(const sensor_msgs::Imu::ConstPtr&);
                void rlEncoderCallback(const std_msgs::Int32::ConstPtr&);

                // other functions
                float quatToRad(float, float, float, float);

                // handles
                ros::Subscriber imuSub;
                ros::Subscriber rlEncoderSub;
                ros::NodeHandle* n;

                // imu variables
                double angZ; // [rad]
                double velZ; // [rad/s]
                double accZ; // [m/s^2]

                // encoder variables
                int rlEncoder; // stores rl motor encoder count

};


#endif