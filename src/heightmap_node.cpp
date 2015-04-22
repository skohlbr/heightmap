#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <matrixstore.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

using namespace visualization_msgs;

static const float CELL_SIZE_X = 0.1f;
static const float CELL_SIZE_Y = 0.1f;

msSparseMatrix_t *sm;
ros::Publisher publisher;
std::mutex pub_mutex;
ros::Timer pub_timer;
static const ros::Duration timerDuration {1.0}; // in seconds

int row=0, col=0, num_rows=100, num_cols=100;
double *buf;

void handleInputMessage(const sensor_msgs::PointCloud& msg)
{
    for(const auto& point : msg.points) {
        int x = point.x / CELL_SIZE_X;
        int y = point.y / CELL_SIZE_Y;
        double height = point.z;

        double old_height;
        msSparseMatrixRead(sm, &old_height, x, y, 1, 1);

        if (isnan(old_height) || old_height < height)
            msSparseMatrixWrite(sm, &height, 1, 1, x, y);
    }

    pub_timer.start();
}
    
void publish()
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
	pointcloud.header.frame_id = "map";
	
	for(int i=0; i < num_rows; i++) {
		for(int j=0; j < num_cols; j++) {
			int index = i*num_cols + j;

			geometry_msgs::Point32 point;
			point.x = i * CELL_SIZE_X;
			point.y = j * CELL_SIZE_Y;
			point.z = buf[index];
			pointcloud.points.push_back(point);

			printf("%8.5f %8.5f %8.5f\n",
				   point.x, point.y, point.z);
		}
	}

	publisher.publish(pointcloud);
	
	pub_mutex.unlock();
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
    ros::Subscriber sub = nh.subscribe("pointcloud", 256, handleInputMessage);
    publisher = nh.advertise<sensor_msgs::PointCloud>("heightmap_vis", 1, true);

    pub_timer = nh.createTimer(timerDuration,
                               [](const ros::TimerEvent& event) {
                                   publish();
                               });
                                   
    ros::spin();
    return 0;
}

