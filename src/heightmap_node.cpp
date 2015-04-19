#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <matrixstore.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

using namespace visualization_msgs;

static const float SCALE_X = 100.0f;
static const float SCALE_Y = 100.0f;
static const float SCALE_Z = 100.0f;

msSparseMatrix_t *sm;
ros::Publisher marker_array_pub;
std::mutex mutex;
ros::Timer pub_timer;
static const ros::Duration timerDuration {1.0}; // in seconds

int row=0, col=0, num_rows=100, num_cols=100;
double *buf;

void handleInputMessage(const sensor_msgs::PointCloud& msg)
{
	for(const auto& point : msg.points) {
		int x = point.x * SCALE_X;
		int y = point.y * SCALE_Y;
		double height = point.z * SCALE_Z;

		double old_height;
		msSparseMatrixRead(sm, &old_height, x, y, 1, 1);

		if (isnan(old_height) || old_height < height)
			msSparseMatrixWrite(sm, &height, 1, 1, x, y);
	}

	pub_timer.start();
}
	
void publish()
{
	if (!mutex.try_lock())
		return;
	
	ROS_INFO ("Timer ticked: sending MarkerArray message");
	
	if (0 == msSparseMatrixRead(sm, buf,
								row, col,
								num_rows, num_cols)) {
		ROS_ERROR("Couldn't read sparse matrix: %s", msGetError());
		mutex.unlock();
		return;
	}

	MarkerArray marker_array;
	marker_array.markers.emplace_back();

	Marker& marker = marker_array.markers.back();
	marker.header.frame_id = "map";
			
	marker.ns = "heighmap_vis";
	marker.id = 1;
	marker.type = Marker::POINTS;
	marker.action = Marker::MODIFY; // it's add-or-modify

	marker.color.r = 0.1;
	marker.color.g = 0.2;
	marker.color.b = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
			
	marker.points.reserve(num_rows * num_cols);
			
	for(int i=0; i < num_rows; i++) {
		for(int j=0; j < num_cols; j++) {
			int index = i * num_cols + j;
			if (isnan(buf[index]))
				continue;

			marker.points.emplace_back();
			geometry_msgs::Point& point = marker.points.back();
					
			point.x = i / SCALE_X;
			point.y = j / SCALE_Y;
			point.z = buf[index] / SCALE_Z;
			printf("%8.5f %8.5f %8.5f\n",
				   point.x, point.y, point.z);
		}
	}

	ROS_INFO("~ Sent %ld markers", marker.points.size());
	marker_array_pub.publish(marker_array);

	mutex.unlock();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmap_server");
	const char *sm_path = "/tmp/heightmap-store/";

	sm = msSparseMatrixOpen(sm_path, 64, 64);
	if (!sm) {
		ROS_ERROR("Failed to open sparse matrix `%s`: %s",
				  sm_path, msGetError());
		return false;
	}

	buf = new double[num_cols * num_rows];
	
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("pointcloud", 256, handleInputMessage);
	marker_array_pub = nh.advertise<MarkerArray>("heightmap_vis", 1, true);

	pub_timer = nh.createTimer(timerDuration,
							   [](const ros::TimerEvent& event) {
								   publish();
							   },
							   true); // oneshot
	
	ros::spin();
	return 0;
}

