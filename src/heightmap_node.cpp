#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <memory>

#include <matrixstore.h>
#include <ros/console.h>
#include <ros/ros.h>
#include "heightmap/Query.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

template<typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args&& ...cons_args) {
	return std::unique_ptr<T>(new T(cons_args...));
}

using namespace visualization_msgs;

static const float CELL_SIZE_X = 0.1f;
static const float CELL_SIZE_Y = 0.1f;

msSparseMatrix_t *sm;
std::string map_frame;

std::unique_ptr<tf::TransformListener> tf_listener;

ros::Publisher publisher;
std::mutex pub_mutex;
ros::Timer pub_timer;
int row=0, col=0, num_rows=100, num_cols=100;
double *buf;

void publishVisualization()
{
	if (!pub_mutex.try_lock())
		return;
	
    ROS_INFO ("Timer ticked: sending message");
    
    if (0 == msSparseMatrixRead(sm, buf, row, col, num_rows, num_cols)) {
        ROS_ERROR("Couldn't read sparse matrix: %s", msGetError());
		pub_mutex.unlock();
        return;
    }

	sensor_msgs::PointCloud pointcloud;
	pointcloud.header.frame_id = map_frame;
	
	for(int i=0; i < num_rows; i++) {
		for(int j=0; j < num_cols; j++) {
			int index = i*num_cols + j;

			geometry_msgs::Point32 point;
			point.x = i * CELL_SIZE_X;
			point.y = j * CELL_SIZE_Y;
			point.z = buf[index];
			pointcloud.points.push_back(point);

			// printf("%8.5f %8.5f %8.5f\n",
			// 	   point.x, point.y, point.z);
		}
	}

	publisher.publish(pointcloud);
	
	pub_mutex.unlock();
}

void handleInputMessage(const sensor_msgs::PointCloud& msg)
{
	const sensor_msgs::PointCloud *msgp = &msg;
	bool transformed = false;
	
	if (msg.header.frame_id != map_frame) {
		transformed = true;
		auto *new_msg = new sensor_msgs::PointCloud();
		tf_listener->transformPointCloud(map_frame, msg, *new_msg);
		msgp = new_msg;
	}
	
    for(const auto& point : msgp->points) {
        int x = point.x / CELL_SIZE_X;
        int y = point.y / CELL_SIZE_Y;
        double height = point.z;

        double old_height;
        msSparseMatrixRead(sm, &old_height, x, y, 1, 1);

        if (isnan(old_height) || old_height < height)
            msSparseMatrixWrite(sm, &height, 1, 1, x, y);
    }

	if (transformed)
		delete msgp;
	
	publishVisualization();
}

bool handleQuery(heightmap::Query::Request &req,
				 heightmap::Query::Response &res)
{
	ROS_DEBUG("serving request");
	
	res.samples = std::move(req.samplePoints);

	for(auto& sample : res.samples) {
	}

	std::string cur_frame_id;
	for(auto& sample : res.samples) {
		if (sample.header.frame_id != map_frame) {
			cur_frame_id = sample.header.frame_id;
			tf_listener->transformPoint(map_frame, sample, sample);
		}
		
		int x = sample.point.x / CELL_SIZE_X;
		int y = sample.point.x / CELL_SIZE_Y;
		double double_val;

		msSparseMatrixRead(sm, &double_val, x, y, 1, 1);
		
		sample.point.z = double_val; // double -> float conversion here

		if (!cur_frame_id.empty()) {
			tf_listener->transformPoint(cur_frame_id, sample, sample);
			cur_frame_id.clear();
		}
	}

	return true;
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
	for(int i=0; i < num_rows*num_cols; i++)
		buf[i] = 0.0f;
	
    ros::NodeHandle nh;

	tf_listener = make_unique<tf::TransformListener>(nh);
	map_frame = "map";
	
    ros::Subscriber sub = nh.subscribe("pointcloud", 256, handleInputMessage);
    publisher = nh.advertise<sensor_msgs::PointCloud>("heightmap_vis", 1, true);

	ros::ServiceServer service = nh.advertiseService("heightmap_query", handleQuery);

    ros::spin();
    return 0;
}

