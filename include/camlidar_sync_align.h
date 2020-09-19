#ifndef _CAMLIDAR_SYNC_ALIGN_H_
#define _CAMLIDAR_SYNC_ALIGN_H_

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream> // for lidar pcd files

#include <ros/ros.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// keyboard input tool
#include "keyinput.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// topic synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

using namespace std;
inline string dtos(double x) {
	stringstream s;
	s << setprecision(6) << fixed << x;
	return s.str();
};

inline string itos(double x) {
	stringstream s;
	s << x;
	return s.str();
};

class CamLidarSyncAlign {
public:
    CamLidarSyncAlign(ros::NodeHandle& nh);
    ~CamLidarSyncAlign();

// private variables

    // node handler
    ros::NodeHandle nh_;
    ros::Subscriber sub1, sub2;


     // subscribers
    message_filters::Subscriber<sensor_msgs::Image> *img_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sub;

    message_filters::Synchronizer<MySyncPolicy> *sync_sub;

    // topic names
    string topicname_img_;
    string topicname_lidar_;

    // data container (buffer)
    cv::Mat buf_img_; // Images from mvBlueCOUGAR-X cameras.
    double buf_time_; // triggered time stamp from Arduino. (second)
    pcl::PointCloud<pcl::PointXYZI>::Ptr buf_lidar_; // point clouds (w/ intensity) from Velodyne VLP16 
    
    float* buf_lidar_x;
    float* buf_lidar_y;
    float* buf_lidar_z;
    float* buf_lidar_intensity;
    unsigned short* buf_lidar_ring;
    float* buf_lidar_time;
    int n_pts_lidar;

// private methods
public:
    void callbackImageLidarSync(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_lidar);
    void imgcb(const sensor_msgs::ImageConstPtr& msg_image);
    void lidarcb(const sensor_msgs::PointCloud2ConstPtr& msg_lidar);

};


/* implementation */
CamLidarSyncAlign::CamLidarSyncAlign(ros::NodeHandle& nh)
: nh_(nh)
{   
    cout << " ALGINER STARTS.\n";

    // initialize image container & subscribers.
    topicname_img_ = "/0/image_raw";
    
    // initialize lidar container & subscribers.
    buf_lidar_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	buf_lidar_x = new float[100000];
	buf_lidar_y = new float[100000];
	buf_lidar_z = new float[100000];
	buf_lidar_intensity = new float[100000];
	buf_lidar_ring = new unsigned short[100000];
	buf_lidar_time = new float[100000];
	n_pts_lidar = 0;

    topicname_lidar_ = "/lidar0/velodyne_points";

    sub1 = nh_.subscribe("/0/image_raw",10,&CamLidarSyncAlign::imgcb,this);
    sub2 = nh_.subscribe("/lidar0/velodyne_points",10,&CamLidarSyncAlign::lidarcb,this);

    this->img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topicname_img_, 10);
    this->lidar_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, topicname_lidar_, 10);
    
    // Generate topic synchronizer
    this->sync_sub = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*this->img_sub, *this->lidar_sub);
    this->sync_sub->registerCallback(boost::bind(&CamLidarSyncAlign::callbackImageLidarSync,this, _1, _2));

};
CamLidarSyncAlign::~CamLidarSyncAlign(){
    delete[] buf_lidar_x;
    delete[] buf_lidar_y;
    delete[] buf_lidar_z;
    delete[] buf_lidar_intensity;
    delete[] buf_lidar_ring;
    delete[] buf_lidar_time;
};
void CamLidarSyncAlign::callbackImageLidarSync(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_lidar){
    // get image
    cout << " camlidar node callback.\n";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
    buf_img_ = cv_ptr->image;

	double time_img = (double)(msg_image->header.stamp.sec * 1e6 + msg_image->header.stamp.nsec / 1000) / 1000000.0;
    cout << "Callback time (image): " << time_img << "\n";
    double time_lidar = (double)(msg_lidar->header.stamp.sec * 1e6 + msg_lidar->header.stamp.nsec / 1000) / 1000000.0;
    cout << "Callback time (lidar): " << time_lidar << "\n";

    // get width and height of 2D point cloud data
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

       memcpy(buf_lidar_x+i, &msg_lidar->data[arrayPosX], sizeof(float));
       memcpy(buf_lidar_y+i, &msg_lidar->data[arrayPosY], sizeof(float));
       memcpy(buf_lidar_z+i, &msg_lidar->data[arrayPosZ], sizeof(float));
       memcpy(buf_lidar_intensity+i, &msg_lidar->data[ind_intensity], sizeof(float));
       memcpy(buf_lidar_ring+i, &msg_lidar->data[ind_ring], sizeof(unsigned short));
       memcpy(buf_lidar_time+i, &msg_lidar->data[ind_time], sizeof(float));
    }

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = buf_lidar_;
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();
    cout <<"n_pts lidar: " <<n_pts<<endl;
};
void CamLidarSyncAlign::imgcb(const sensor_msgs::ImageConstPtr& msg_image){
 // get image
    cout << " cam  callback.\n";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
    buf_img_ = cv_ptr->image;

	double time_img = (double)(msg_image->header.stamp.sec * 1e6 + msg_image->header.stamp.nsec / 1000) / 1000000.0;
    cout << "Callback time (image): " << time_img << "\n";
}

void CamLidarSyncAlign::lidarcb(const sensor_msgs::PointCloud2ConstPtr& msg_lidar){
 double time_lidar = (double)(msg_lidar->header.stamp.sec * 1e6 + msg_lidar->header.stamp.nsec / 1000) / 1000000.0;
    cout << "Callback time (lidar): " << time_lidar << "\n";


}
#endif
