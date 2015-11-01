/*
 * switch.cpp
 * switches between behaviors.
 * DETECTOR runs all the time, except for reseting
 * END OF HALLWAY at all times
 * HALLWAY > get LA > TURN > update LA > HALLWAY > RESET > HALLWAY
 *
 *  Created on: Jun 15, 2014
 *      Author: aida
 */


#include "ros/ros.h"
#include "aida/Velocity.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "visualization_msgs/Marker.h"

#include <sstream>

#include "aida/GetDirection.h"
#include "aida/Walls.h"
#include "aida/People.h"
#include "aida/Update.h"
#include "constants.h"
#include "definitions.h"

ros::Publisher run_status, vel_pub, direction_pub, marker_pub;
ros::Subscriber detector_sub, hallway_sub, turn_sub, reset_sub, end_sub, walls_sub;
ros::ServiceClient get_direction_srv, update_srv;

std_msgs::Bool turn_direction;
aida::Velocity velocity;

bool status[STEPS], failed, was_in_center, wall_is_visible[2];
aida::Point last_location;
aida::Line walls[2];
float walls_dis;

void update_marker(const char*, int);

struct {
	int location, user_id, direction;
} current_user;

void publish_run_status()
{
	status[DETECTOR] = status[HALLWAY] || status[TURN];

	std_msgs::UInt8MultiArray msg;
	msg.data.clear();
	for (int i = 0; i < STEPS; i++)
		msg.data.push_back(status[i]);

	run_status.publish(msg);

	std::string st = "";
	if (status[HALLWAY])
		st = "hallway";
	else if (status[TURN])
		st = (turn_direction.data == LEFT) ? "turning left" : "turning right";
	else if (status[RESET])
		st = "reset", STATUS_TEXT;

	update_marker(st.c_str(), STATUS_TEXT);
	ROS_DEBUG("Status : %s", st.c_str());
}

void publish_velocity()
{	//TODO should I set the velocity.valid = false after publishing it once?
	if (velocity.valid &&
			aida::abs(velocity.vel.linear.x) < 5 &&
			aida::abs(velocity.vel.linear.y) < 5 &&
			aida::abs(velocity.vel.linear.z) < 5 &&
			aida::abs(velocity.vel.angular.x) < 5 &&
			aida::abs(velocity.vel.angular.y) < 5 &&
			aida::abs(velocity.vel.angular.z) < 5)
		vel_pub.publish(velocity.vel);
	else
		ROS_ERROR("Invalid velocity.");

	std::stringstream ss;
	ss << "(" << velocity.vel.linear.x << "," << velocity.vel.linear.y << "," << velocity.vel.linear.z << "), ("
			<< velocity.vel.angular.x << "," << velocity.vel.angular.y << "," << velocity.vel.angular.z << ")";
	update_marker(ss.str().c_str(), VELOCITY_TEXT);

	ROS_DEBUG("Velocity: linear (x=%.2f,y=%.2f,z=%.2f), angular (x=%.2f,y=%.2f,z=%.2f)",
			velocity.vel.linear.x, velocity.vel.linear.y, velocity.vel.linear.z,
			velocity.vel.angular.x, velocity.vel.angular.y, velocity.vel.angular.z);
}

void publish_turn_direction()
{
	direction_pub.publish(turn_direction);
}

int getLocation(float x, float y)
{
	if (!wall_is_visible[LEFT] || !wall_is_visible[RIGHT]) {
		ROS_WARN("At least one of the walls is not visible. Cannot get location.");
		return -1;
	}

	float left = walls[LEFT].slope * x + walls[LEFT].y;
	float right = walls[RIGHT].slope * x + walls[RIGHT].y;

	int location = ((y-left) * NUM_LOCATIONS) / (right-left);
	if (location < 0 || location >= NUM_LOCATIONS) {
		ROS_WARN("Location is out of boundaries. location=%d", location);
		return -1;
	}

	ROS_DEBUG("user at location %d", location);
	return location;
}

void update_marker(const char* str, int id)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = FRAME_ID;
	marker.header.stamp = ros::Time::now();
	marker.ns = "switch";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.scale.z = 0.5;

	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	marker.id = id;

	marker.pose.position.x = 0;
	marker.pose.position.y = 2;
	if (id == STATUS_TEXT)
		marker.pose.position.z = 5;
	else if (id == ERROR_MSG)
		marker.pose.position.z = 4.5;
	else if (id == VELOCITY_TEXT)
		marker.pose.position.z = 4;
	else
		marker.pose.position.z = 0;

	marker.text = str;
	marker_pub.publish(marker);
}

int human_frame_count;
void detectorCallback(const aida::People::ConstPtr& msg)
{
	if (msg->data.size() == 0) {
		human_frame_count = 0;
		return;
	}

	if (status[HALLWAY]) {
		human_frame_count++;
		ROS_DEBUG("Seen human in %d frame(s), waiting for 4.", human_frame_count);
	}

	float wpos_y = msg->data[0].pos_y - ((walls[LEFT].getY(msg->data[0].pos_x) + walls[RIGHT].getY(msg->data[0].pos_x)) / 2);

	if (status[HALLWAY] && human_frame_count > 4) {
		aida::GetDirection srv;
		srv.request.user_id = 0;	//change to use actual user_id in future!
		current_user.user_id = 0;
		current_user.location = getLocation(msg->data[0].pos_x, msg->data[0].pos_y);

		if (current_user.location >= 0) {
			srv.request.location = current_user.location;
			get_direction_srv.call(srv);
			current_user.direction = turn_direction.data = srv.response.direction;

			failed = false;
			was_in_center = false;

			update_marker("no errors yet", ERROR_MSG);

			status[HALLWAY] = false;
			status[TURN] = true;
			human_frame_count = 0;

			ROS_INFO("Starting turn");
		}
	} else if (status[TURN]) {
		if (was_in_center && aida::abs(wpos_y) > (walls_dis - 0.5) / 3) {
			if (turn_direction.data == LEFT && wpos_y > 0) {
				ROS_INFO("Expecting the user to turn right, but she got too close to the left wall.");
				update_marker("too close to left", ERROR_MSG);
				failed = true;
			}
			if (turn_direction.data == RIGHT && wpos_y < 0) {
				ROS_INFO("Expecting the user to turn left, but she got too close to the right wall.");
				update_marker("too close to right", ERROR_MSG);
				failed = true;
			}
		}
	}

	if (status[TURN]) {
		last_location.x = msg->data[0].pos_x;
		last_location.y = msg->data[0].pos_y;
		last_location.z = msg->data[0].pos_z;

		if (aida::abs(wpos_y) <= (walls_dis - 0.5) / 3)
			was_in_center = true;
	}
}

void hallwayCallback(const aida::Velocity::ConstPtr& msg)
{
	if (status[HALLWAY])
		velocity = *msg;
}

void resetCallback(const aida::Velocity::ConstPtr& msg)
{
	if (status[RESET]) {
		if (!msg->valid) {
			status[RESET] = false;
			status[HALLWAY] = true;

			ROS_INFO("Starting hallway centering.");
		} else {
			velocity = *msg;
		}
	}
}

void turnCallback(const aida::Velocity::ConstPtr& msg)
{
	if (status[TURN]) {
		velocity = *msg;

		if (!msg->valid) {
			status[TURN] = false;

			if (last_location.y > 0 && turn_direction.data == LEFT) {
				failed = true;
				ROS_INFO("Expecting the user to turn right, but she passed on the left side.");
				update_marker("expected right, passed on left", ERROR_MSG);
			}

			if (last_location.y < 0 && turn_direction.data == RIGHT) {
				failed = true;
				ROS_INFO("Expecting the user to turn left, but she passed on the right side.");
				update_marker("expected left, passed on right", ERROR_MSG);
			}

			aida::Update msg;
			msg.request.location = current_user.location;
			msg.request.direction = current_user.direction;
			msg.request.user_id = current_user.user_id;
			msg.request.error = failed;
			update_srv.call(msg);

			status[HALLWAY] = true;

			ROS_INFO("Starting hallway centering.");
		}
	}
}

void wallsCallback(const aida::Walls::ConstPtr& msg)
{
	if (msg->equ.size() != 2 || msg->is_visible.size() != 2) {
		ROS_WARN("Invalid message received at walls subscriber.");
		return;
	}

	walls[LEFT].slope = msg->equ[LEFT].slope;
	walls[LEFT].y = msg->equ[LEFT].y;
	wall_is_visible[LEFT] = msg->is_visible[LEFT];

	walls[RIGHT].slope = msg->equ[RIGHT].slope;
	walls[RIGHT].y = msg->equ[RIGHT].y;
	wall_is_visible[RIGHT] = msg->is_visible[RIGHT];

	if (wall_is_visible[LEFT] && wall_is_visible[RIGHT])
		walls_dis = aida::dis(walls[LEFT], walls[RIGHT]);
	else
		walls_dis = -1;
	ROS_DEBUG("Walls are %f apart.", walls_dis);
}

void endCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if (msg->data && status[HALLWAY]) {
		status[HALLWAY] = false;
		status[RESET] = true;

		ROS_INFO("Starting 180 turn.");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "switch");
	ros::NodeHandle n;

	status[HALLWAY] = true;
	status[TURN] = false;

	detector_sub = n.subscribe("people", 1000, detectorCallback);
	hallway_sub = n.subscribe("hallway", 1000, hallwayCallback);
	turn_sub = n.subscribe("turn", 1000, turnCallback);
	walls_sub = n.subscribe("walls", 1000, wallsCallback);
	reset_sub = n.subscribe("reset", 1000, resetCallback);
	end_sub = n.subscribe("end_of_hallway", 1000, endCallback);

	get_direction_srv = n.serviceClient<aida::GetDirection>("get_direction");
	update_srv = n.serviceClient<aida::Update>("result");

	run_status = n.advertise<std_msgs::UInt8MultiArray>("run_status", 1000);
	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	direction_pub = n.advertise<std_msgs::Bool>("turn_direction", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok()) {
		publish_turn_direction();
		publish_velocity();
		publish_run_status();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
