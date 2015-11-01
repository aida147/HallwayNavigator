#include "ros/ros.h"
#include "ros/package.h"

#include "aida/GetDirection.h"
#include "aida/Update.h"
#include "constants.h"

#include<iostream>
#include<fstream>
#include<vector>
using namespace std;

struct Record {
	int user_id;
	float weight[NUM_LOCATIONS][2];
};

fstream file;
vector<Record> cache;

int get_cache_index(int user_id)
{
	for (int i = 0; i < (int) cache.size(); i++)
		if (cache[i].user_id == user_id)
			return i;
	return -1;
}

bool in_cache(int user_id)
{
	return get_cache_index(user_id) >= 0;
}

Record get_from_cache(int user_id)
{
	return cache[get_cache_index(user_id)];
}

void write_to_cache(Record r)
{
	cache[get_cache_index(r.user_id)] = r;
}

void write(Record r)
{
	if (in_cache(r.user_id))
		write_to_cache(r);

	file.seekp(r.user_id * sizeof(Record));
	file.write((char*)&r, sizeof(Record));
	if (file.fail())
		ROS_ERROR("Failed to write record to file. user_id=%d", r.user_id);
}

Record read(int user_id)
{
	Record r;
	if (in_cache(user_id))
		return get_from_cache(user_id);

	file.seekg(user_id * sizeof(Record));
	file.read((char*)&r, sizeof(Record));
	if (file.fail())
		ROS_ERROR("Failed to read record. user_id=%d", r.user_id);
	else
		cache.push_back(r);
	return r;
}

bool getDirection(aida::GetDirection::Request &req,
		aida::GetDirection::Response &res)
{
	ROS_INFO("user = %d, location = %d", req.user_id, req.location);
	Record r = read(req.user_id);
	if (r.weight[req.location][LEFT] > r.weight[req.location][RIGHT])
		res.direction = LEFT;
	else if (r.weight[req.location][LEFT] < r.weight[req.location][RIGHT])
		res.direction = RIGHT;
	else
		res.direction = rand()%2;
	ROS_INFO(res.direction == LEFT ? "Direction to turn: left" : "Direction to turn: right");
	ROS_DEBUG("%f %f", r.weight[req.location][LEFT], r.weight[req.location][RIGHT]);
	return true;
}

float increase(float x)
{
	return x > 0.25 ? sqrt(x) : 2 * x;
}

float decrease(float x)
{
	return x / 2;
}

bool update(aida::Update::Request &req,
		aida::Update::Response &res)
{
	Record r = read(req.user_id);
	bool c = req.direction;
	if (!req.error)
		if (req.direction == RIGHT)
			for (int i = req.location; i < NUM_LOCATIONS; i++)
				r.weight[i][RIGHT] = increase(r.weight[i][RIGHT]);
		else
			for (int i = 0; i <= req.location; i++)
				r.weight[i][LEFT] = increase(r.weight[i][LEFT]);
	else
		if (req.direction == RIGHT)
			for (int i = 0; i <= req.location; i++)
				r.weight[i][RIGHT] = decrease(r.weight[i][RIGHT]);
		else
			for (int i = req.location; i < NUM_LOCATIONS; i++)
				r.weight[i][LEFT] = decrease(r.weight[i][LEFT]);

	write(r);
	return true;
}

int main(int argc, char **argv)
{
	std::string path = ros::package::getPath("aida");
	file.open((path + "/data").c_str(), ios::in | ios::out | ios::binary);
	ROS_INFO("%s", (path + "/data").c_str());
	if (file.fail())
		ROS_FATAL("Cannot open data file for learning algorithm.");

	ros::init(argc, argv, "la");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("get_direction", getDirection);
	ros::ServiceServer update_sub = n.advertiseService("result", update);

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok()) {
		file.flush();

		ros::spinOnce();
		loop_rate.sleep();
	}

	file.close();
	return 0;
}
