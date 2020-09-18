#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"


#include <sensor_msgs/TimeReference.h> // arduino time
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;

void pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar){
    // get width and height of 2D point cloud data
    cout <<"t: "<<msg_lidar->header.stamp.sec<<" [s] / "<<msg_lidar->header.stamp.nsec << "\n";

    for(int i = 0; i < msg_lidar->width; i++) {
       int arrayPosX = i*msg_lidar->point_step + msg_lidar->fields[0].offset; // X has an offset of 0
       int arrayPosY = i*msg_lidar->point_step + msg_lidar->fields[1].offset; // Y has an offset of 4
       int arrayPosZ = i*msg_lidar->point_step + msg_lidar->fields[2].offset; // Z has an offset of 8

       int ind_intensity = i*msg_lidar->point_step + msg_lidar->fields[3].offset; // 12
       int ind_ring = i*msg_lidar->point_step + msg_lidar->fields[4].offset; // 16
       int ind_time = i*msg_lidar->point_step + msg_lidar->fields[5].offset; // 18

       float X = 0.0;
       float Y = 0.0;
       float Z = 0.0;
       float intensity = 0.0;
       unsigned short ring = 0.0;
       float time = 0.0;

       
       //cout << "xyz intensity ring time: "<<*(buf_lidars_x[id]+i)<<","<<*(buf_lidars_y[id]+i)<<","<<*(buf_lidars_z[id]+i)
       //<<","<<*(buf_lidars_intensity[id]+i)<<","<<*(buf_lidars_ring[id]+i)<<","<<*(buf_lidars_time[id]+i)<<endl;
    }
}

void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar){
	pointcloud2tobuffers(msg_lidar);
};
// Get current data/time, format is yyyy-mm-dd.hh:mm:ss
const std::string currentDateTime(){
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about data/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H_%M_%S", &tstruct);

    return buf;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camlidar_module");
    ros::NodeHandle nh("~");
    ros::Subscriber subs_lidars_; // from Velodyne lidars
    subs_lidars_ = nh.subscribe<sensor_msgs::PointCloud2>("/lidar0/velodyne_points", 1, &callbackLidar);

    while(ros::ok()){
        ros::spinOnce();
	}

	ROS_INFO_STREAM("End of the program.\n");
	return -1;
}
