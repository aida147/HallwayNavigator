/*
 * hallway.cpp
 * Navigates the robot to the center of the hallway
 * if argument rotate is passed, it will use the rotating algorithm,
 * otherwise will use the sweep line algorithm
 *
 *  Created on: Jul 1, 2014
 *      Author: aida
 */

#include<vector>
#include<utility>
#include<cmath>

#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "aida/Velocity.h"
#include "std_msgs/UInt8MultiArray.h"
#include "aida/Walls.h"
#include <visualization_msgs/Marker.h>

#include "constants.h"
#include "definitions.h"


const int V = 5;	//number of previous velocities to keep for averaging
int tail;			//for averaging
int algo;			//ROTATE or SWEEP

ros::Subscriber scan_sub;
ros::Publisher vel_pub, marker_pub, wall_pub;
geometry_msgs::Twist velocity[V];

aida::Point goal;	//the point in the center we are trying to reach
aida::Line res[3];	//res[LEFT], res[RIGHT], res[CENTER]
bool visible[2];	//visible[LEFT], visible[RIGHT]

bool cmp(aida::Point a, aida::Point b)
{
	return a.y < b.y;
}

std::vector<aida::Point> convert(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::vector<aida::Point> p;

	float ang = msg->angle_min;
	for (int i = 0; i < (int) msg->ranges.size(); i++) {
		if (!isnan(msg->ranges[i])) {
			aida::Point t;
			t.x = msg->ranges[i] * cos(ang);
			t.y = msg->ranges[i] * sin(ang);
			if (abs(t.x) < 5)
				p.push_back(t);
		}
		ang+= msg->angle_increment;
	}

	return p;
}

void rotate(std::vector<aida::Point> p)
{
	ROS_DEBUG("Find walls, using rotating algorithm.");
	int best = 0;
	float best_ang = 0, best_ly, best_ry;
	float range = M_PI/2;
	for (float ang = range; ang > -range; ang-= 0.05) {
		float m = tan(ang);

		std::vector<aida::Point> t;
		t.resize(p.size());

		for (int i = 0; i < (int) p.size(); i++) {
			t[i].x = p[i].x * cos(ang) - p[i].y * sin(ang);
			t[i].y = p[i].x * sin(ang) + p[i].y * cos(ang);
		}

		sort(t.begin(), t.end(), cmp);

		float ry = t[0].y, ly = t[t.size()-1].y;
		int l = 0, r = 0;
		for (int i = 0; i < (int) t.size(); i++) {
			if (ly-t[i].y < 0.05)
				l++;
			else if (t[i].y-ry < 0.05)
				r++;
		}

		if (l+r > best) {
			best = l+r;
			best_ang = ang;

			best_ly = (0 * sin(-ang) + ly * cos(-ang)) - tan(-best_ang) * (0 * cos(-ang) - ly * sin(-ang));
			best_ry = (0 * sin(-ang) + ry * cos(-ang)) - tan(-best_ang) * (0 * cos(-ang) - ry * sin(-ang));
		}
	}

	res[LEFT].slope = res[RIGHT].slope = tan(-best_ang);
	res[LEFT].y = best_ly;
	res[RIGHT].y = best_ry;
	visible[LEFT] = visible[RIGHT] = true;

	if (abs(best_ly-best_ry) < 1) {
		if (best_ly > 0)	//only left
			visible[RIGHT] = false;
		else
			visible[LEFT] = false;
	}
}

void sweep(std::vector<aida::Point> p)
{
	ROS_DEBUG("Find walls, using sweep line algorithm.");
	float range = M_PI/2 - 0.1;

	//LEFT
	for (float ang = range; ang > -range; ang-= 0.05) {
		float m = tan(ang);
		float b = -m*p[p.size()-1].x + p[p.size()-1].y;
		int cnt = 0;
		for (int i = 0; i < (int) p.size() - 1; i++)
			if (p[i].x > p[p.size()-1].x && m*p[i].x - p[i].y + b < 0)
				cnt++;
		if (cnt >= 10) {
			res[LEFT].slope = m;
			res[LEFT].y = b;
			break;
		}
	}

	//RIGHT
	for (float ang = -range; ang < range; ang+= 0.05) {
		float m = tan(ang);
		float b = -m*p[0].x + p[0].y;
		int cnt = 0;
		for (int i = 1; i < (int) p.size(); i++)
			if (p[i].x > p[0].x && m*p[i].x - p[i].y + b > 0)
				cnt++;
		if (cnt >= 10) {
			res[RIGHT].slope = m;
			res[RIGHT].y = b;
			break;
		}
	}

	visible[LEFT] = visible[RIGHT] = true;
	if (aida::abs(atan2(res[LEFT].slope,1)-atan2(res[RIGHT].slope,1)) > 0.1 ||	//TODO 0.3?
			aida::abs(res[LEFT].y-res[RIGHT].y) < 1) {	//only one wall
		if (p[0].x > p[p.size()-1].x)//left wall
			visible[RIGHT] = false;
		else
			visible[LEFT] = false;
	}
}

void findWalls(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::vector<aida::Point> p = convert(msg);
	if (algo == ROTATE)
			rotate(p);
	else {
		if (algo != SWEEP)
			ROS_WARN("whaaat? :D\nChoose a valid algorithm for finding walls. Using sweep line.");
		sweep(p);
	}

	geometry_msgs::Twist vel;

	if (visible[LEFT] && visible[RIGHT]) {
		float cy = (res[LEFT].y + res[RIGHT].y)/2;
		goal.x = aida::abs(cy) < 0.1 ? 2 : aida::abs(cy) * 4;
		goal.y = (res[LEFT].slope + res[RIGHT].slope)/2 * goal.x + cy;

		vel.linear.x = aida::abs(cy) >= 0.2 ? 0.1 : aida::abs(cy) / 2;
		ROS_DEBUG("Both walls visible.");
	} else {
		goal.x = 2;
		goal.y = res[visible[LEFT] ? LEFT : RIGHT].slope * goal.x;

		vel.linear.x = 0;
		ROS_DEBUG(visible[LEFT] ? "Only left wall visible." : "Only right wall visible.");
	}

	vel.linear.x = std::max(vel.linear.x, 0.05);

	res[CENTER].slope = goal.y / goal.x;
	res[CENTER].y = 0;

	vel.angular.z = 2 * atan2(res[CENTER].slope, 1);
	if (aida::abs(vel.angular.z) <= 0.05)
		vel.angular.z = 0;

	velocity[tail] = vel;
	ROS_DEBUG("Immediate velocity: angular=(%f,%f,%f), linear(%f,%f,%f)",
			vel.angular.x, vel.angular.y, vel.angular.z,
			vel.linear.x, vel.linear.y, vel.linear.z);
	tail = (tail+1)%V;
}

void draw()
{
	visualization_msgs::Marker l[3];
	for (int i = 0; i < 3; i++) {
		l[i].header.frame_id = FRAME_ID;//"/camera_depth_frame";
		l[i].header.stamp = ros::Time::now();
		l[i].ns = "hallway_search";
		l[i].action = visualization_msgs::Marker::ADD;
		l[i].pose.orientation.w = 1.0;
		l[i].type = visualization_msgs::Marker::LINE_STRIP;
		l[i].scale.x = 0.1;
		l[i].id = i;
	}


	l[LEFT].color.b = 1.0;
	l[LEFT].color.a = 1.0 * visible[LEFT];

	l[CENTER].color.g = 1.0;
	l[CENTER].color.a = 1.0;

	l[RIGHT].color.b = 1.0;
	l[RIGHT].color.a = 1.0 * visible[RIGHT];

	for (int i = 0; i < 2; i++) {
		geometry_msgs::Point p1, p2;
		p1.x = 0;
		p1.y = res[i].y;
		p1.z = 0;
		l[i].points.push_back(p1);

		p2.x = 2;
		p2.y = 2*res[i].slope + res[i].y;
		p2.z = 0;
		l[i].points.push_back(p2);

		marker_pub.publish(l[i]);
	}

	{
		geometry_msgs::Point p1, p2;
		p1.x = 0;
		p1.y = 0;
		p1.z = 0;
		l[CENTER].points.push_back(p1);

		p2.x = goal.x;
		p2.y = goal.y;
		p2.z = 0;
		l[CENTER].points.push_back(p2);

		marker_pub.publish(l[CENTER]);
	}
}

void publishVelocity()
{
	aida::Velocity msg;
	msg.vel;
	for (int i = 0; i < V; i++) {
		msg.vel.angular.z+= velocity[i].angular.z;
		msg.vel.linear.x+= velocity[i].linear.x;
	}
	msg.vel.angular.z/= V;
	msg.vel.linear.x/= V;
	msg.valid = true;
	ROS_DEBUG("Velocity angular.z= %.2f, linear.x= %.2f", msg.vel.angular.z, msg.vel.linear.x);
	vel_pub.publish(msg);
}

void publishWalls()
{
	aida::Walls msg;
	msg.equ.resize(2);
	msg.is_visible.resize(2);

	for (int i = 0; i < 2; i++) {
		msg.equ[i].slope = res[i].slope;
		msg.equ[i].y = res[i].y;
		msg.is_visible[i] = visible[i];
	}

	wall_pub.publish(msg);
}

int main(int argc, char **argv)
{
	if (argc > 1 && aida::equ(argv[1], "rotate"))
		algo = ROTATE;
	else
		algo = SWEEP;	//default

	ros::init(argc, argv, "hallway");
	ros::NodeHandle n;
	scan_sub = n.subscribe("scan", 1000, findWalls);
	vel_pub = n.advertise<aida::Velocity>("hallway", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	wall_pub = n.advertise<aida::Walls>("walls", 1000);
	tail = 0;

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		publishVelocity();
		publishWalls();
		draw();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
