/*
 * reset.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: aida
 */

#include "ros/ros.h"
#include "aida/Velocity.h"
#include "std_msgs/UInt8MultiArray.h"

#include "constants.h"

ros::Subscriber status_sub;
ros::Publisher vel_pub;
bool running;
ros::Time start;

void statusCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
	if (msg->data[RESET]) {
		if (running) {
			ros::Duration passed = ros::Time::now() - start;
			if (start.sec != 0 && passed.toSec() >= 2) {
				aida::Velocity v;
				v.valid = false;
				vel_pub.publish(v);
			}
		} else {
			running = true;
			start = ros::Time::now();
		}
	} else {
		running = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reset");
	ros::NodeHandle n;

	status_sub = n.subscribe("run_status", 1000, statusCallback);
	vel_pub = n.advertise<aida::Velocity>("reset", 1000);

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



