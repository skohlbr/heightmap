
#include <cmath>
#include <cstdio>
#include <iostream>

#include <heightmap/sparse_block_matrix.hpp>
#include <heightmap/Query.h>
#include <heightmap/pointcloud_iterator.hpp>

#include <geometry_msgs/Point.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


using namespace heightmap;

template<typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args&& ... cons_args) {
	return std::unique_ptr<T>(new T(cons_args...));
}

static const float CELL_SIZE_X = 0.1f;
static const float CELL_SIZE_Y = 0.1f;

SparseMatrix<double> h { {64, 64}, NAN };
Matrix<double> vis_buf = Matrix<double>::create(100, 100);

const std::string map_frame = "map";
std::unique_ptr<tf::TransformListener> tf_listener;
ros::Publisher vis_publisher;

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
	ROS_INFO ("Timer ticked: sending message");

	h.read({0, 0}, vis_buf);

	sensor_msgs::PointCloud2 pointcloud;
	pointcloud.header.frame_id = map_frame;
	pointcloud.is_bigendian = false;
	pointcloud.point_step = 12;
	pointcloud.is_dense = true;
	pointcloud.width = vis_buf.rows() * vis_buf.cols();
	pointcloud.height = 1;
	pointcloud.row_step = pointcloud.width * pointcloud.point_step;

	setPointCloudFields(pointcloud);

	const size_t rawDataSize = vis_buf.rows() * vis_buf.cols() * 3;
	float *rawData = new float[rawDataSize];

	int raw_index = 0;
	for(int i=0; i < vis_buf.rows(); i++) {
		for(int j=0; j < vis_buf.cols(); j++) {
			rawData[raw_index++] = i * CELL_SIZE_X; // X
			rawData[raw_index++] = j * CELL_SIZE_Y; // Y
			rawData[raw_index++] = vis_buf[{i, j}];     // Z

			// printf("%8.5f %8.5f %8.5f\n",
			// 	   x, y, z);
		}
	}

	pointcloud.data.assign((uint8_t*) rawData,
						   (uint8_t*) rawData+rawDataSize);

	vis_publisher.publish(pointcloud);
}

void updateHeightmap(const sensor_msgs::PointCloud2& msg)
{
	auto it = begin<float>(msg);
	auto it_end = end<float>(msg);

	unsigned int iteratedOver = 0;
	unsigned int updateCount = 0;

	for(; it != it_end; it++) {
		iteratedOver++;
		heightmap::Point<float>& point = *it;

		if(isnan(point.x) || isnan(point.y) || isnan(point.z))
			continue;

		int x = point.x / CELL_SIZE_X;
		int y = point.y / CELL_SIZE_Y;
		double height = point.z;
		double old_height = h[{x, y}];
		if (isnan(old_height) || old_height < height) {
			h[{x, y}] = height;
			updateCount++;
		}

	}

	ROS_INFO("Iterated over %d, updated %d cells", iteratedOver, updateCount);
	publishVisualization();
}

void updateHeightmap(const octomap::OcTree& octree)
{
	auto iter = octree.begin_leafs();
	auto iter_end = octree.end_leafs();

	auto write_if_higher = [](double current, double new_value) {
		return isnan(current) || new_value > current;
	};

	for(; iter != iter_end; iter++) {
		// An OcTree leaf is a cube.
		auto center = iter.getCoordinate();
		auto size = iter.getSize();

		double x_min = floor(center.x() - size/2);
		double y_min = floor(center.y() - size/2);

		double x_max = ceil(center.x() + size/2);
		double y_max = ceil(center.y() + size/2);

		double z_max = center.z() + size/2;

		Region<> region {
			{ (int)y_max, (int)x_min },
			{ (int)y_max - (int)y_min,
			  (int)x_max - (int)x_min }
		};

		h.fill(region, center.z() + size/2,
		       write_if_higher);
	}
}

void handlePointCloudMessage(const sensor_msgs::PointCloud2& msg)
{
	if (msg.header.frame_id == map_frame) {
		ROS_WARN("Using identity transform");
		updateHeightmap(msg);
		return;
	}

	auto new_msg = make_unique<sensor_msgs::PointCloud2>();
	if(!pcl_ros::transformPointCloud(map_frame, msg, *new_msg, *tf_listener)) {
		ROS_ERROR("Couldn't transform point cloud from message => discarding");
		return;
	}

	updateHeightmap(*new_msg);
}

void handleOctomapMessage(const octomap_msgs::Octomap& msg)
{
	if (msg.header.frame_id != map_frame) {
		ROS_ERROR("Received Octomap is in wrong tf frame: is `%s', must be `%s' => discarding",
		          msg.header.frame_id.c_str(), map_frame.c_str());
		return;
	}

	if (!msg.binary) {
		ROS_ERROR("Received Octomap is of the wrong type: "
		          "is full occupancy, must be binary => discarding");
		return;
	}

	std::unique_ptr<octomap::OcTree> octree {octomap_msgs::binaryMsgToMap(msg)};
	updateHeightmap(*octree);
}

bool handleQuery(heightmap::Query::Request &req,
				 heightmap::Query::Response &res)
{
	ROS_DEBUG("serving request");

	bool need_transform = (req.corner.header.frame_id != map_frame);

	const float x_step = req.x_size / (req.x_resolution - 1);
	const float y_step = req.y_size / (req.y_resolution - 1);

	res.x_size = req.x_resolution;
	res.y_size = req.y_resolution;
	res.map.clear();
	res.map.resize(req.x_resolution * req.y_resolution);

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
			res.map[index] = h[{sx, sy}];
		}
	}

	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heightmap_node");

    ros::NodeHandle nh;

    tf_listener = make_unique<tf::TransformListener>(nh);
    ros::Subscriber pointcloud_sub = nh.subscribe("pointcloud2", 256, handlePointCloudMessage);
    ros::Subscriber octomap_sub = nh.subscribe("octomap", 16, handleOctomapMessage);

    ros::ServiceServer service = nh.advertiseService("heightmap_query", handleQuery);
    vis_publisher = nh.advertise<sensor_msgs::PointCloud2>("heightmap_vis", 1, true);

    ROS_INFO("Listening to %s for PointClouds", pointcloud_sub.getTopic().c_str());
    ROS_INFO("Listening to %s for Octomaps", octomap_sub.getTopic().c_str());
    ros::spin();
    return 0;
}
