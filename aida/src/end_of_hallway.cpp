/*
 * end_of_hallway.cpp
 * Detects if the end of the hallway is at most 5.5 meters far
 * Assumes the robot is facing the center
 *
 *  Created on: Jul 1, 2014
 *      Author: aida
 */

#include<vector>
#include<utility>
#include<cmath>
#include<algorithm>

#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8MultiArray.h"
#include <visualization_msgs/Marker.h>

#include "constants.h"
#include "definitions.h"

ros::Subscriber scan_sub;
ros::Publisher end_pub, marker_pub;
aida::Point end[2];
bool visible;

void findWall(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::vector<aida::Point> p;

	float ang = msg->angle_min;
	for (int i = 0; i < (int) msg->ranges.size(); i++) {
		if (!isnan(msg->ranges[i]))
			p.push_back(aida::Point(msg->ranges[i] * cos(ang), msg->ranges[i] * sin(ang)));
		ang+= msg->angle_increment;
	}

	//assuming the robot is centered, find the point in front of the robot
	float x = -1;
	for (int i = 0; i < (int) p.size(); i++)
		if (aida::abs(p[i].y) < 0.1)
			x = p[i].x;

	//count how many other points share roughly the same X
	int cnt = 0;
	float miny = 0, maxy = 0;
	for (int i = 0; i < (int) p.size(); i++)
		if (aida::abs(p[i].x-x) < 0.2) {
			miny = std::min(miny, p[i].y);
			maxy = std::max(maxy, p[i].y);
			cnt++;
		}
	ROS_DEBUG("%d point(s) belong to the wall at the end of hallway.", cnt);

	visible = false;
	if (cnt > 200 && x > 2 && x < 5.5) {	//to help with hallway_search (that one cuts at 5)
		end[0].x = end[1].x = x;
		end[0].y = miny;
		end[1].y = maxy;
		visible = true;
	}

	std_msgs::Bool out;
	out.data = visible;
	end_pub.publish(out);
}

void draw()
{
	visualization_msgs::Marker l;
	for (int i = 0; i < 3; i++) {
		l.header.frame_id = FRAME_ID;//"/camera_depth_frame";
		l.header.stamp = ros::Time::now();
		l.ns = "end_of_hallway";
		l.action = visualization_msgs::Marker::ADD;
		l.pose.orientation.w = 1.0;
		l.type = visualization_msgs::Marker::LINE_STRIP;
		l.scale.x = 0.1;
	}

	l.id = 0;
	l.color.r = 1.0;
	l.color.g = 1.0;
	l.color.a = 1.0 * visible;

	geometry_msgs::Point p1, p2;
	p1.x = end[0].x;
	p1.y = end[0].y;
	p1.z = 0;
	l.points.push_back(p1);

	p2.x = end[1].x;
	p2.y = end[1].y;
	p2.z = 0;
	l.points.push_back(p2);

	marker_pub.publish(l);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "end_of_hallway");
	ros::NodeHandle n;
	scan_sub = n.subscribe("scan", 1000, findWall);
	end_pub = n.advertise<std_msgs::Bool>("end_of_hallway", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok()) {
		draw();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
