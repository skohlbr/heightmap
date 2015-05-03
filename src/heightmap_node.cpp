#include "heightmap/Query.h"
#include "heightmap/pointcloud_iterator2.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <memory>

#include <matrixstore.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

template<typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args&& ... cons_args) {
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

void setPointCloudFields(sensor_msgs::PointCloud2& pointcloud) {
	pointcloud.fields.clear();

	sensor_msgs::PointField xField;
	xField.name='x';
	xField.offset = 0;
	xField.datatype = sensor_msgs::PointField::FLOAT32;
	xField.count = 1;
	sensor_msgs::PointField yField;
	yField.name='y';
	yField.offset = 4;
	yField.datatype = sensor_msgs::PointField::FLOAT32;
	yField.count = 1;
	sensor_msgs::PointField zField;
	zField.name='z';
	zField.offset = 8;
	zField.datatype = sensor_msgs::PointField::FLOAT32;
	zField.count = 1;

	pointcloud.fields.push_back(xField);
	pointcloud.fields.push_back(yField);
	pointcloud.fields.push_back(zField);
}

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

	sensor_msgs::PointCloud2 pointcloud;
	pointcloud.header.frame_id = map_frame;
	pointcloud.is_bigendian = false;
	pointcloud.point_step = 12;
	pointcloud.is_dense = true;
	pointcloud.width = num_cols;
	pointcloud.height = num_rows;
	pointcloud.row_step= pointcloud.width * pointcloud.point_step;

	setPointCloudFields(pointcloud);

	const size_t rawDataSize = num_rows * num_cols * 3;
	float *rawData = new float[rawDataSize];
	
	for(int i=0; i < num_rows; i++) {
		for(int j=0; j < num_cols; j++) {
			int index = i*num_cols + j;
			rawData[index*3]     = i * CELL_SIZE_X; // X
			rawData[index*3 + 1] = j * CELL_SIZE_Y; // Y
			rawData[index*3 + 2] = buf[index];      // Z
			
			// printf("%8.5f %8.5f %8.5f\n",
			// 	   x, y, z);
		}
	}

	pointcloud.data.assign((uint8_t*) rawData,
						   (uint8_t*) rawData+rawDataSize);

	publisher.publish(pointcloud);

	pub_mutex.unlock();
}

void handleInputMessage(const sensor_msgs::PointCloud2& msg)
{
	const sensor_msgs::PointCloud2 *msgp = &msg;
	bool transformed = false;

	if (msg.header.frame_id != map_frame) {
		transformed = true;
		sensor_msgs::PointCloud2 *new_msg = new sensor_msgs::PointCloud2();
		
		if(pcl_ros::transformPointCloud(map_frame,
										msg, *new_msg,
										*tf_listener)) {
			msgp = new_msg;
		} else {
			ROS_WARN("Using identity transform");
			msgp = &msg;
		}
	}

	auto it = heightmap::begin<float>(*msgp);
	auto it_end = heightmap::end<float>(*msgp);
	
	unsigned int iteratedOver = 0;
	unsigned int updateCount = 0;
	unsigned int skippedNans = 0;

	for(; it != it_end; it++) {
		iteratedOver++;
		heightmap::Point<float> point = *it;

		if(isnan(point.x) || isnan(point.y) || isnan(point.z)) {
			skippedNans++;
			continue;
		}

		int x = point.x / CELL_SIZE_X;
		int y = point.y / CELL_SIZE_Y;
		double height = point.z;

		double old_height;

		msSparseMatrixRead(sm, &old_height, x, y, 1, 1);
		if (isnan(old_height) || old_height < height) {
			msSparseMatrixWrite(sm, &height, 1, 1, x, y);
			updateCount++;
		}

	}

//   ROS_INFO("Iterated over %d, updated %d cells", iteratedOver, updateCount);

	publishVisualization();
}

bool handleQuery(heightmap::Query::Request &req,
				 heightmap::Query::Response &res)
{
	ROS_DEBUG("serving request");

	bool need_transform = (req.corner.header.frame_id != map_frame);

	const float x_step = req.x_size / (req.x_resolution - 1);
	const float y_step = req.y_size / (req.y_resolution - 1);

	std::vector<double> submap;
	submap.resize(req.x_resolution * req.y_resolution);

	for(int i=0; i < req.y_resolution; i++) {
		for(int j=0; j < req.x_resolution; j++) {
			geometry_msgs::PointStamped point = req.corner;
			point.point.x += x_step * i;
			point.point.y += y_step * j;

			if (need_transform)
				tf_listener->transformPoint(map_frame, point, point);

			int sx = point.point.x / CELL_SIZE_X;
			int sy = point.point.y / CELL_SIZE_Y;
			int index = req.x_resolution * i + j;
			msSparseMatrixRead(sm, &submap[index], sx, sy, 1, 1);
		}
	}

	res.x_size = req.x_resolution;
	res.y_size = req.y_resolution;
	res.map = std::move(submap);

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

    ros::Subscriber sub = nh.subscribe("pointcloud2", 256, handleInputMessage);
    publisher = nh.advertise<sensor_msgs::PointCloud2>("heightmap_vis", 1, true);

    ros::ServiceServer service = nh.advertiseService("heightmap_query", handleQuery);

    ROS_INFO("Listening to %s", sub.getTopic().c_str());
    ros::spin();
    return 0;
}

