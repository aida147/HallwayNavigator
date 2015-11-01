#include<iostream>
#include<vector>
#include<utility>
#include<cmath>

#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8MultiArray.h"
#include <visualization_msgs/Marker.h>

#include "aida/Walls.h"
#include "constants.h"
#include "definitions.h"

const int V = 5;

ros::Subscriber scan_sub;
ros::Publisher vel_pub, marker_pub, wall_pub;
std::vector<aida::Point> p;
float velocity[V];
int tail;
aida::Line res[3];

aida::Line getLine(int s, int e)	//[s, e)
{
	int Count = e-s;
	float SumX = 0;
	float SumY = 0;
	float SumX2 = 0; 	//sum of the squares of the x values
	float SumXY = 0;	//sum of the products x*y for all the points

	for (int i = s; i < e; i++) {
		SumX+= p[i].x;
		SumY+= p[i].y;
		SumX2+= p[i].x * p[i].x;
		SumXY+= p[i].x * p[i].y;
	}

	aida::Line res;
	float XMean = SumX / Count;
	float YMean = SumY / Count;
	if (SumX2 - SumX * XMean < EPS) {	//Slope == INF
		res.isVertical = true;
		res.slope = INF;
	} else {
		res.slope = (SumXY - SumX * YMean) / (SumX2 - SumX * XMean);
	}
	res.y = YMean - res.slope * XMean;

	return res;
}

bool cmp(aida::Point a, aida::Point b)
{
	return a.y < b.y;
}

void findWalls(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	p.resize(0);

	float ang = msg->angle_min;
	for (int i = 0; i < (int) msg->ranges.size(); i++) {
		if (!isnan(msg->ranges[i])) {
			aida::Point t;
			t.x = msg->ranges[i] * cos(ang);
			t.y = msg->ranges[i] * sin(ang);
			if (t.x < 5)
				p.push_back(t);
		}
		ang+= msg->angle_increment;
	}
	sort(p.begin(), p.end(), cmp);

	float best = INF;
	int n = p.size();
	for (int m = n/10; m < 9*n/10; m++) {	// [0, m) , [m,n)
		aida::Line r = getLine(0, m), l = getLine(m, n);
		if (isnan(l.slope) || isnan(r.slope))
			continue;

		if (aida::abs(atan2(l.slope, 1) - atan2(r.slope, 1)) < best) {
			best = aida::abs(atan2(l.slope, 1) - atan2(r.slope, 1));
			res[LEFT] = l;
			res[RIGHT] = r;
		}
	}

	aida::Point goal;
	goal.x = 2;	//TODO
	float avg_slope = (res[LEFT].slope + res[RIGHT].slope)/2;

	if (aida::abs(res[LEFT].y-res[RIGHT].y) < 0.2) {
		if (aida::abs(avg_slope) < 0.1) {	// Parallel to walls
			if (res[LEFT].y < 0)	// only right wall
				goal.y = avg_slope * goal.x + 0.5;	//TODO
			else					// only left wall
				goal.y = avg_slope * goal.x - 0.5;	//TODO
		} else {
			goal.y = avg_slope * goal.x;
		}
	} else {
		goal.y = avg_slope * goal.x + (res[LEFT].y + res[RIGHT].y)/2;
	}

	res[CENTER].slope = goal.y / goal.x;
	res[CENTER].y = 0;

	std::string s;
	double vel = 2 * atan2(res[CENTER].slope, 1);
	if (aida::abs(vel) <= 0.1) {
		s = "go forward";
		vel = 0;
	} else if (vel < 0)
		s = "turn right";
	else if (vel > 0)
		s = "turn left";
	velocity[tail] = vel;
	tail = (tail+1)%V;
//	ROS_INFO("%.2f\t%s", vel, s.c_str());
}

void draw()
{
	visualization_msgs::Marker l[3];
	for (int i = 0; i < 3; i++) {
		l[i].header.frame_id = "/camera_depth_frame";
		l[i].header.stamp = ros::Time::now();
		l[i].ns = "points_and_lines";
		l[i].action = visualization_msgs::Marker::ADD;
		l[i].pose.orientation.w = 1.0;
		l[i].type = visualization_msgs::Marker::LINE_STRIP;
		l[i].scale.x = 0.1;
	}

	l[LEFT].id = 0;
	l[LEFT].color.b = 1.0;
	l[LEFT].color.a = 1.0;

	l[CENTER].id = 1;
	l[CENTER].color.g = 1.0;
	l[CENTER].color.a = 1.0;

	l[RIGHT].id = 2;
	l[RIGHT].color.r = 1.0;
	l[RIGHT].color.a = 1.0;

	marker_pub.publish(l[LEFT]);

	for (int i = 0; i < 3; i++) {
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
}

void publishVelocity()
{
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.angular.z = 0;
	for (int i = 0; i < V; i++)
		msg.angular.z+= velocity[i];
	msg.angular.z/= V;
//	ROS_INFO("Velocity angular.z= %.2f", msg.angular.z);
	vel_pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bestFit");
	ros::NodeHandle n;
	scan_sub = n.subscribe("scan", 1000, findWalls);
//	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	vel_pub = n.advertise<geometry_msgs::Twist>("hallway", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>
		("visualization_marker", 10);
	tail = 0;

	ros::Rate loop_rate(10);
	while (ros::ok()) {
		publishVelocity();
		draw();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
