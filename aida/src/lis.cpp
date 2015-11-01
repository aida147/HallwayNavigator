#include<iostream>
#include<vector>
#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
using namespace std;

ros::Subscriber sub;
ros::Publisher pub;
float velocity;

const float INF = 10000;
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
int lis(vector<float> a, int& end);

int lis(vector<float> a, int& end)
{
	int n = a.size();
	float s[n+1];
	int d[n];
	
	for (int i = 0; i <= n; i++)
		s[i] = INF;
	s[0] = -INF;

	for (int i = 0; i < n; i++) {
		d[i] = upper_bound(s, s+n, a[i]) - s;
		s[d[i]] = min(s[d[i]], a[i]);
	}

	int ans = 0;
	for (int i = 0; i < n; i++)
		if (d[i] > ans) {
			end = i;
			ans = d[i];
		}
	return ans;
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	vector<float> a, ranges;
	float prev = msg->range_min;
	ranges = msg->ranges;
/*	for (int i = 0; i < (int) ranges.size(); i++) {
		if (isnan(ranges[i]) && prev < 9)
			ranges[i] = prev;
		prev = ranges[i];
	}*/

	for (int i = (int) ranges.size()-1; i >= 0; i--) {
		if (ranges[i] >= msg->range_min &&
				ranges[i] <= msg->range_max) {
		//	a.push_back(ranges[i]);
			a.push_back(((int)(ranges[i]*100))/100.0);
			//cout << ranges[i] << "\n";
		}
	}
//	cout << endl;
	//exit(0);

	int n = a.size();
	int le;
	int l = lis(a, le);

	vector<float> ra;
	for (int i = n-1; i >= 0; i--)
		ra.push_back(a[i]);
	int re;
	int r = lis(ra, re);
	re = n-re;

	string s = "";
	if (abs(l-r) <= n*0.05) {
		if (le <= n*0.7) {
			s = "go forward";
			velocity = 0;
		} else {
			s = "turn right/left";
			velocity = l < r ? 2.0 : -2.0;
		}
	} else if (l > r) {
		s = "turn right";
		velocity = -2.0 * (l-r)/n;
	} else if (l < r) {
		s = "turn left";
		velocity = 2.0 * (r-l)/n;
	}

	ROS_DEBUG("%d,%d-%d:%d-%d,%f", n, l, le, r, re, velocity);
	ROS_INFO("%s", s.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "findWalls");
	ros::NodeHandle n;
	sub = n.subscribe("scan", 1000, chatterCallback);
	pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		geometry_msgs::Twist msg;
		msg.linear.x = 0;
		msg.angular.z = velocity;
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
//	ros::spin();
	return 0;
}
