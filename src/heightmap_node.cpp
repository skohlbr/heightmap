#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <matrixstore.h>
#include <cstdio>
#include <cmath>

msSparseMatrix_t *sm = NULL;

static const float SCALE_X = 100.0f;
static const float SCALE_Y = 100.0f;
static const float SCALE_Z = 100.0f;

void msgCallback(const sensor_msgs::PointCloud& msg)
{
	for(const auto& point : msg.points) {
		int x = point.x * SCALE_X;
		int y = point.y * SCALE_Y;
		double height = point.z * SCALE_Z;
			
		double old_height;
		msSparseMatrixRead(sm, &old_height, x, y, 1, 1);

		if (isnan(old_height) || old_height < height) {
			msSparseMatrixWrite(sm, &height, 1, 1, x, y);
			fprintf(stderr, " *");
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmap_node");

	sm = msSparseMatrixOpen("/tmp/heightmap-store/", 64, 64);
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("pointcloud", 256, msgCallback);
	ros::spin();

	return 0;
}
