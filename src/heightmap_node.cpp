#include <cmath>
#include <cstdio>
#include <iostream>

#include <heightmap/sparse_block_matrix.hpp>
#include <heightmap/Query.h>
#include <heightmap/pointcloud_iterator.hpp>
#include <heightmap/image.hpp>

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


// Measurement unit for both is points/unit-of-length
static const float RESOLUTION_X = 10;
static const float RESOLUTION_Y = 10;

static const float CELL_SIZE_X = 1. / RESOLUTION_X;
static const float CELL_SIZE_Y = 1. / RESOLUTION_Y;

static const std::string map_frame = "map";
std::unique_ptr<tf::TransformListener> tf_listener;

SparseMatrix<double> h { {64, 64}, NAN };
Matrix<double> vis_buf = Matrix<double>::create(100, 100);
ros::Publisher vis_publisher;


void publishVisualization();
void handlePointCloudMessage(const sensor_msgs::PointCloud2& msg);
void handleOctomapMessage(const octomap_msgs::Octomap& msg);
bool handleQuery(heightmap::Query::Request &req, heightmap::Query::Response &res);
bool loadFile(const std::string &filename);
bool parseArgs(char** arg, char** end);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "heightmap_node");
	ros::NodeHandle nh;

	if (!parseArgs(&argv[1], &argv[argc]))
		return 1;

	tf_listener = make_unique<tf::TransformListener>(nh);
	auto pcl_sub = nh.subscribe("pointcloud2", 16, handlePointCloudMessage);
	auto octomap_sub = nh.subscribe("octomap", 16, handleOctomapMessage);
	/// If we don't keep this variable around, the service gets deleted
	/// right before proceeding to the next statement
	auto service = nh.advertiseService("heightmap", handleQuery);
	vis_publisher = nh.advertise<sensor_msgs::PointCloud2>("heightmap_vis", 1, true);

	ROS_INFO("Listening to %s for PointClouds", pcl_sub.getTopic().c_str());
	ROS_INFO("Listening to %s for Octomaps", octomap_sub.getTopic().c_str());
	ROS_INFO("Visualization is published to %s", vis_publisher.getTopic().c_str());

	ros::Timer timer = nh.createTimer(ros::Duration(1.0),
	                                  [](const ros::TimerEvent&) {
		                                  publishVisualization();
	                                  });
	ros::spin();
	return 0;
}

void printUsage()
{
	fprintf(stderr, "Usage: heightmap_node [load <filename>]\n");
	fprintf(stderr, "   load <filename>\t- Load an image as the initial heightmap\n");
}

bool loadFile(const std::string &filename)
{
	Image img = Image::loadFile(filename);
	if (img.isNull())
		return false;

	MatrixRef<double*> map = img.data();
	ROS_WARN("Loaded image (%dx%d) will be mapped onto the heightmap with "
	         "resolution %fx%f points/unit-of-length",
	         map.cols(), map.rows(), RESOLUTION_X, RESOLUTION_Y);

	h.write(map, Index<> {0, 0});
	return true;
}

bool parseArgs(char **arg, char **end)
{
	for (; arg != end; arg++) {
		if (0 == strcmp(*arg, "-h") || 0 == strcmp(*arg, "--help")) {
			printUsage();
			return false;
		} else if (0 == strcmp(*arg, "--load") || 0 == strcmp(*arg, "-l")) {
			arg++;
			if (arg == end) {
				fprintf(stderr, "`--load' requires an argument\n");
				return false;
			}

			if (!loadFile(*arg))
				return false;
		} else {
			fprintf(stderr, "Unrecognized argument: `%s'\n", *arg);
			return false;
		}
	}

	return true;
}

void setPointCloudFields(sensor_msgs::PointCloud2& pointcloud)
{
	pointcloud.fields.clear();

	int i = 0;
	for (std::string&& name : {"x", "y", "z"}) {
		sensor_msgs::PointField field;
		field.name = std::move(name);
		field.offset = i * sizeof(float);
		field.datatype = sensor_msgs::PointField::FLOAT32;
		field.count = 1;
		pointcloud.fields.push_back(field);
		i++;
	}
}

void publishVisualization()
{
	ROS_INFO ("Timer ticked: publishing visualization");

	h.read({0, 0}, vis_buf);

	sensor_msgs::PointCloud2 pointcloud;
	pointcloud.header.frame_id = map_frame;
	pointcloud.is_bigendian = false;
	pointcloud.point_step = 3 * sizeof(float);
	pointcloud.is_dense = true;
	pointcloud.width = vis_buf.rows() * vis_buf.cols();
	pointcloud.height = 1;
	pointcloud.row_step = pointcloud.width * pointcloud.point_step;

	setPointCloudFields(pointcloud);

	std::vector<float> raw_data;
	raw_data.reserve(3 * vis_buf.rows() * vis_buf.cols());

	for(int i=0; i < vis_buf.rows(); i++) {
		for(int j=0; j < vis_buf.cols(); j++) {
			raw_data.push_back(i * CELL_SIZE_X);  // X
			raw_data.push_back(j * CELL_SIZE_Y);  // Y
			raw_data.push_back(vis_buf[{i, j}]);  // Z
		}
	}

	uint8_t* bytes_ptr = (uint8_t*) raw_data.data();
	size_t n_bytes = raw_data.size() * sizeof(float);
	pointcloud.data.assign(bytes_ptr, bytes_ptr+n_bytes);
	vis_publisher.publish(pointcloud);
}

void updateHeightmap(const sensor_msgs::PointCloud2& msg)
{
	auto iter = begin<float>(msg);
	auto iter_end = end<float>(msg);

	for(; iter != iter_end; iter++) {
		heightmap::Point<float>& point = *iter;

		if(isnan(point.x) || isnan(point.y) || isnan(point.z))
			continue;

		int x = point.x / CELL_SIZE_X;
		int y = point.y / CELL_SIZE_Y;
		double height = point.z;
		double old_height = h[{x, y}];
		if (isnan(old_height) || old_height < height)
			h[{x, y}] = height;
	}

	ROS_INFO("Visualization published: %ld points",
	         std::distance(iter, iter_end));
	publishVisualization();
}

void updateHeightmap(const octomap::OcTree& octree)
{
	auto iter = octree.begin_leafs();
	auto iter_end = octree.end_leafs();

	auto is_higher = [](double current, double new_value) {
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

		// Fill the region with value, only where it is higher than
		// the existing value
		h.fill(region, z_max, is_higher);
	}
}

void handlePointCloudMessage(const sensor_msgs::PointCloud2& msg)
{
	if (msg.header.frame_id == map_frame) {
		updateHeightmap(msg);
		return;
	}

	// The point cloud must first be transformed. pcl_ros's API forces
	// us to allocate a new PointCloud2 object
	auto new_msg = make_unique<sensor_msgs::PointCloud2>();
	if (!pcl_ros::transformPointCloud(map_frame, msg, *new_msg, *tf_listener)) {
		ROS_ERROR("Couldn't transform point cloud from message => discarding");
		return;
	}

	updateHeightmap(*new_msg);
}

void handleOctomapMessage(const octomap_msgs::Octomap& msg)
{
	if (msg.header.frame_id != map_frame) {
		ROS_ERROR("Received Octomap has is in the wrong tf frame: "
		          "is `%s', should be `%s' => discarding",
		          msg.header.frame_id.c_str(), map_frame.c_str());
		return;
	}

	if (!msg.binary) {
		ROS_ERROR("Received Octomap is of the wrong type: "
		          "is full occupancy information, must be binary => discarding");
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

	if (req.x_samples == 0 || req.y_samples == 0)
		return false;

	const float x_step = req.x_size / (req.x_samples - 1);
	const float y_step = req.y_size / (req.y_samples - 1);

	res.x_samples = req.x_samples;
	res.y_samples = req.y_samples;
	res.map.clear();
	res.map.resize(req.x_samples * req.y_samples);

	for(int i=0; i < req.y_samples; i++) {
		for(int j=0; j < req.x_samples; j++) {
			geometry_msgs::PointStamped point;
			point.point.x = req.corner.point.x + x_step * i;
			point.point.y = req.corner.point.y + y_step * j;

			if (need_transform)
				tf_listener->transformPoint(map_frame, point, point);

			int sx = point.point.x / CELL_SIZE_X;
			int sy = point.point.y / CELL_SIZE_Y;
			int index = req.x_samples * i + j;

			res.map[index] = h[{sx, sy}];
		}
	}

	return true;
}
