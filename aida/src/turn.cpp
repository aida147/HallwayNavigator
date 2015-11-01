/*
 * turn.cpp
 *
 *  Created on: Jun 20, 2014
 *      Author: viki
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
	if (msg->data[TURN]) {
		if (running) {
			ros::Duration passed = ros::Time::now() - start;
			if (start.sec != 0 && passed.toSec() >= 8) {
				aida::Velocity v;
				v.vel.linear.x = 0;
				v.vel.linear.y = 0;
				v.vel.linear.z = 0;
				v.vel.angular.x = 0;
				v.vel.angular.y = 0;
				v.vel.angular.z = 0;
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
	ros::init(argc, argv, "turn");
	ros::NodeHandle n;

	status_sub = n.subscribe("run_status", 1000, statusCallback);
	vel_pub = n.advertise<aida::Velocity>("turn", 1000);

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
