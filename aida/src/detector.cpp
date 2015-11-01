/*
 * detector.cpp
 * Finds humans (any thing that fits in a cube with certain dimensions!) and walls
 * Assumes it can see both walls, ceiling and floor
 *
 *  Created on: Jun 17, 2014
 *      Author: aida
 */

#include <cmath>

#include "definitions.h"
#include "constants.h"
#include "aida/Human.h"
#include "aida/People.h"
#include "aida/Walls.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/UInt8MultiArray.h"

#include <Eigen/Core>

#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

bool running;
ros::Publisher rotated_cloud_pub, people_pub;
ros::Publisher vis_human[MAX_TRACKER];
geometry_msgs::PolygonStamped zeropoly;

void draw_human(aida::Point, aida::Point, aida::Point, int);

void boundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, aida::Point& min_, aida::Point& max_)
{	//returns min/max for (x,y,z) of all the points in the point cloud
	aida::Point n, x;
	n.z = x.z = cloud->points[0].z;
	n.y = x.y = cloud->points[0].y;
	n.x = x.x = cloud->points[0].x;

	for (int i = 1; i < (int) cloud->points.size(); i++) {
		n.z = min(n.z, cloud->points[i].z);
		n.y = min(n.y, cloud->points[i].y);
		n.x = min(n.x, cloud->points[i].x);
		x.z = max(x.z, cloud->points[i].z);
		x.y = max(x.y, cloud->points[i].y);
		x.x = max(x.x, cloud->points[i].x);
	}

	min_ = n;
	max_ = x;
}

void pre(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_d (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	//down sample
	cloud_d->height = cloud->height/10;
	cloud_d->width = cloud->width/10;
	cloud_d->header.frame_id = FRAME_ID;
	cloud_d->points.resize(cloud_d->height * cloud_d->width);
	for (int i = 0; i < cloud_d->height; i++)
		for (int j = 0; j < cloud_d->width; j++)
			cloud_d->points[i * cloud_d->width + j] = cloud->points[i*10*cloud->width + j*10];

	//rotate
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion q;
	q.setW(0.5);
	q.setX(-0.5);
	q.setY(0.5);
	q.setZ(-0.5);
	transform.setRotation(q);
	pcl_ros::transformPointCloud(*cloud_d, *cloud, transform);

	//remove floor, ceiling
	pcl::PointXYZ temp;
	temp.x = temp.y = temp.z = nanf("");
	for (int i = 0; i < (int) cloud->points.size(); i++)
		if (cloud->points[i].z < 0.01 || cloud->points[i].z > 2)
			cloud->points[i] = temp;
}

void removeWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{	//remove walls
	for (int i = 0; i < 2; i++) {
		if (cloud->size() < 200)
			return;

		//fit a plane
		pcl::PointIndices::Ptr inliersp (new pcl::PointIndices);
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
		ransac.setDistanceThreshold (.1);	//TODO
		ransac.setMaxIterations(5000);
		ransac.computeModel();
		ransac.getInliers(inliersp->indices);

		pcl::PointXYZ temp;
		temp.x = temp.y = temp.z = nanf("");

		for (int i = 0; i < (int) inliersp->indices.size(); i++)
			cloud->points[inliersp->indices[i]] = temp;
	}
}

aida::People myFindPeople(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	aida::People ans;

	pcl::PointCloud<pcl::PointXYZ>::CloudVectorType clusters;
	boost::shared_ptr<std::vector<pcl::PointIndices> > labelsIndices(new std::vector<pcl::PointIndices>());
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>()), labels2(new pcl::PointCloud<pcl::Label>());

	pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>::Ptr comparator(new pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>());

	std::vector<bool> plane_labels;
	plane_labels.resize(2);
	plane_labels[0] = true;
	plane_labels[1] = false;
	for (int i = 0; i < (int) cloud->points.size(); i++) {
		pcl::Label t;
		if (isnan(cloud->points[i].x))
			t.label = 0;
		else
			t.label = 1;
		labels2->push_back(t);
	}


	comparator->setDistanceThreshold(0.25f, false);
	comparator->setInputCloud(cloud);
	comparator->setLabels(labels2);
	comparator->setExcludeLabels(plane_labels);

	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZ, pcl::Label> segmenter(comparator);
	segmenter.setInputCloud(cloud);
	segmenter.segment(*labels, *labelsIndices);

	for (size_t i = 0; i < labelsIndices->size(); i++) {
		if (labelsIndices->at(i).indices.size() > 10) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud (*cloud, labelsIndices->at(i).indices, *cluster);

			aida::Point min_, max_, center;
			boundaries(cluster, min_, max_);

			for (int i = 0; i < (int) cluster->points.size(); i++) {
				center.x+= cluster->points[i].x / cluster->points.size();
				center.y+= cluster->points[i].y / cluster->points.size();
				center.z+= cluster->points[i].z / cluster->points.size();
			}

			//the model is basically a cube, that should have a certain height, width, and width/height ratio
			if (min_.x < 9 &&	//behind that it's probably just noise
					(max_.y - min_.y)/(max_.z - min_.z) > 0.1 &&
					(max_.y - min_.y)/(max_.z - min_.z) < 0.8 &&
					(max_.z - min_.z) < 1.9 &&
					(max_.z - min_.z) > 1 &&
					(max_.y - min_.y) < 1 &&
					(max_.y - min_.y) > 0.3 &&
					(max_.x - min_.x) < 0.7) {
				aida::Human l;
				l.pos_x = center.x;
				l.pos_y = center.y;
				l.pos_z = center.z;

				ans.data.push_back(l);
				draw_human(min_, max_, center, ans.data.size()-1);
			}
		}
	}

	return ans;
}

void depthCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_old)
{
	aida::People people;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (!running)
		goto end;

	*cloud = *cloud_old;
	pre(cloud);
	removeWalls(cloud);
	rotated_cloud_pub.publish(*cloud);

	if (cloud->size() < 100)	//the scene is empty
		goto end;


	people = myFindPeople(cloud);
	people_pub.publish(people);

	end:
	for (int i = people.data.size(); i < MAX_TRACKER; i++)
		vis_human[i].publish(zeropoly);
}

void statusCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
	running = msg->data[DETECTOR];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detector");
	ros::NodeHandle n;

	ros::Subscriber cloud_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("/camera/depth/points",1, depthCallback);
	ros::Subscriber status_sub = n.subscribe("run_status", 1000, statusCallback);

	rotated_cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("rotated_cloud", 1);
	people_pub = n.advertise<aida::People> ("people", 1000);

	for (int i = 0; i < MAX_TRACKER; i++) {
		std::string poly = "human_" + boost::lexical_cast<std::string>(i);
		vis_human[i] = n.advertise<geometry_msgs::PolygonStamped>(poly, 1);
	}

	zeropoly.header.frame_id = FRAME_ID;
	zeropoly.polygon.points.push_back(geometry_msgs::Point32());
	zeropoly.polygon.points.push_back(geometry_msgs::Point32());
	zeropoly.polygon.points.push_back(geometry_msgs::Point32());

	running = false;

	ros::Rate loop_rate(LOOP_RATE);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void draw_human(aida::Point min_, aida::Point max_, aida::Point center, int ind)
{
	geometry_msgs::PolygonStamped poly;
	poly.header.frame_id = FRAME_ID;
	poly.polygon.points.resize(26);

	int tail = 0;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = center.x;
	poly.polygon.points[tail].y = center.y;
	poly.polygon.points[tail].z = center.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = center.x;
	poly.polygon.points[tail].y = center.y;
	poly.polygon.points[tail].z = center.z;

	tail++;
	poly.polygon.points[tail].x = max_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = min_.z;

	tail++;
	poly.polygon.points[tail].x = center.x;
	poly.polygon.points[tail].y = center.y;
	poly.polygon.points[tail].z = center.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = max_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = min_.x;
	poly.polygon.points[tail].y = min_.y;
	poly.polygon.points[tail].z = max_.z;

	tail++;
	poly.polygon.points[tail].x = center.x;
	poly.polygon.points[tail].y = center.y;
	poly.polygon.points[tail].z = center.z;

	vis_human[ind].publish(poly);
}
