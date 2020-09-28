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

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/eigen.hpp"

#include <sensor_msgs/Image.h>
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
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    CamLidarSyncAlign(ros::NodeHandle& nh,const string& param_path_,const string& save_path);
    ~CamLidarSyncAlign();
    void saveLidarDataRingTime(const string& file_name);

    void saveSnapshot();
    bool getDataReady(){return this->data_ready_;};

private: // ROS related
    ros::NodeHandle nh_;

     // subscribers
    message_filters::Subscriber<sensor_msgs::Image> *img_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sub;
    message_filters::Synchronizer<MySyncPolicy> *sync_sub;

    // topic names
    string topicname_img_;
    string topicname_lidar_;

    // data container (buffer)
    cv::Mat buf_img_; // Images from mvBlueCOUGAR-X cameras.
    cv::Mat img_undistort_; // image (undistorted)

    double buf_time_; // triggered time stamp from Arduino. (seconds)
    pcl::PointCloud<pcl::PointXYZI>::Ptr buf_lidar_; // point clouds (w/ intensity) from Velodyne VLP16 
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_pcl_warped_; // Pointclouds (warped onto the image frame from the LiDAR frame)
    float* buf_lidar_x;
    float* buf_lidar_y;
    float* buf_lidar_z;
    float* buf_lidar_intensity;
    unsigned short* buf_lidar_ring;
    float* buf_lidar_time;
    int n_pts_lidar;

    int current_seq_; // for snapshot saving.
    bool data_ready_;
    string save_dir_;

private: // image undistorter & lidar warping.
    cv::Mat cvK; // intrinsic matrix of the camera.
    cv::Mat cvDistortion; // distortion parameters (five). {k1 k2 p1 p2 k3}
    cv::Mat cvT_cl;

    Eigen::Matrix3d K;
    Eigen::Matrix4d T_cl; // SE(3) transform from a camera frame to a LiDAR frame.
    int n_cols; // image width
    int n_rows; // image height

    cv::Mat undist_map_x; // undistortion map x (pre-calculated in 'preCalculateUndistortMaps' for speed)
    cv::Mat undist_map_y; // undistortion map y (pre-calculated in 'preCalculateUndistortMaps' for speed)


// private methods
private: // ROS related
    void callbackImageLidarSync(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_lidar);

private: // image undistortion & LiDAR warping
    void readCameraLidarParameter(const string& path_dir);
    void preCalculateUndistortMaps();
    void undistortCurrentImage(const cv::Mat& img_source, cv::Mat& img_dst);

};

/*
*
*
*
*
*/
/* implementation */
CamLidarSyncAlign::CamLidarSyncAlign(ros::NodeHandle& nh, const string& param_path_, const string& save_path)
: nh_(nh), save_dir_(save_path)
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

    this->img_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topicname_img_, 1);
    this->lidar_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, topicname_lidar_, 1);
    
    // Generate topic synchronizer
    this->sync_sub = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*this->img_sub, *this->lidar_sub);
    this->sync_sub->registerCallback(boost::bind(&CamLidarSyncAlign::callbackImageLidarSync,this, _1, _2));

    // Load calibration parameters
    readCameraLidarParameter(param_path_);
    preCalculateUndistortMaps();

    // snapshot parameters
    current_seq_ = 0;
    data_ready_ = false;

    // generate save folder
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    // make image saving directories
    folder_create_command = "mkdir " + save_dir_ + "cam0/";
	system(folder_create_command.c_str());

    // make lidar data saving directories
    folder_create_command = "mkdir " + save_dir_ + "lidar0/";
	system(folder_create_command.c_str());

    // save association
    string file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << "time_us ";
        output_file << "cam0 ";
        output_file << "exposure_us gain_dB ";
        output_file << "lidar0 ";
        output_file << "\n";
    }
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
    //cout << " camlidar node callback.\n";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::BGR8);
    buf_img_ = cv_ptr->image;

    // undistort image
    this->undistortCurrentImage(buf_img_, img_undistort_);

	double time_img = (double)(msg_image->header.stamp.sec * 1e6 + msg_image->header.stamp.nsec / 1000) / 1000000.0;
    double time_lidar = (double)(msg_lidar->header.stamp.sec * 1e6 + msg_lidar->header.stamp.nsec / 1000) / 1000000.0;


    buf_time_ = time_img;

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
    n_pts_lidar = msg_lidar->width; // # of lidar points in one circle

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = buf_lidar_;
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();


    //cout << "Callback time (image): " << time_img << "\n";
    //cout << "Callback time (lidar): " << time_lidar << "\n";
    //cout <<"n_pts lidar: " <<n_pts<<endl;
    
    data_ready_ = true;
};

void CamLidarSyncAlign::readCameraLidarParameter(const string& path_dir){
    // Load calibration files
    cv::FileStorage fsSettings(path_dir, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    
    fsSettings["IMAGE.K"] >> cvK;
    fsSettings["IMAGE.DISTORTION"] >> cvDistortion;
    fsSettings["T_CL"] >> cvT_cl;
    this->n_rows = fsSettings["IMAGE.ROWS"];
    this->n_cols = fsSettings["IMAGE.COLS"];

    cout << cvK<<endl;
    cout << cvDistortion<<endl;
    cout<<cvT_cl<<endl;
    cout<<"n_rows: "<<this->n_rows<<", n_cols: "<<this->n_cols<<endl;
    
    fsSettings.release();

    if (cvK.empty() || cvDistortion.empty() || cvT_cl.empty() || n_rows == 0 || n_cols == 0)
        cerr << "ERROR: Calibration parameters to rectify an image are missing!" << endl;
   
    // Eigen version
    cv::cv2eigen(cvT_cl, T_cl);
    cv::cv2eigen(cvK, K);

    cout << "  Camera & LiDAR information is loaded... DONE!\n";
};

void CamLidarSyncAlign::preCalculateUndistortMaps(){
    // allocation
    undist_map_x = cv::Mat::zeros(n_rows, n_cols, CV_32FC1);
    undist_map_y = cv::Mat::zeros(n_rows, n_cols, CV_32FC1);
    img_undistort_ = cv::Mat::zeros(n_rows, n_cols, CV_8UC3);

    // interpolation grid calculations
    float* map_x_ptr = nullptr;
    float* map_y_ptr = nullptr;
    
    float fu = K(0,0);
    float fv = K(1,1);
    float fuinv = 1.0f/fu;
    float fvinv = 1.0f/fv;
    float centeru = K(0,2);
    float centerv = K(1,2);
    float k1 = cvDistortion.at<double>(0,0);
    float k2 = cvDistortion.at<double>(0,1);
    float p1 = cvDistortion.at<double>(0,2);
    float p2 = cvDistortion.at<double>(0,3);
    float k3 = cvDistortion.at<double>(0,4);

    for(int v = 0; v < n_rows; v++){
        map_x_ptr = undist_map_x.ptr<float>(v);
        map_y_ptr = undist_map_y.ptr<float>(v);

        for(int u = 0; u < n_cols; u++){
            float x = (u-centeru)*fuinv;
            float y = (v-centerv)*fvinv;
            float r2 = x*x + y*y;
            float r4 = r2*r2;
            float r6 = r2*r4;
            float r = sqrtf(r2);
            float r_radial = 1.0f + k1*r2 + k2*r4 + k3*r6;
            float x_dist = x*r_radial + 2*p1*x*y + p2*(r2 + 2*x*x);
            float y_dist = y*r_radial + p1*(r2 + 2*y*y) + 2*p2*x*y;
            *(map_x_ptr+u) = fu*x_dist + centeru;
            *(map_y_ptr+u) = fv*y_dist + centerv;
        }
    }
};

void CamLidarSyncAlign::undistortCurrentImage(const cv::Mat& img_source, cv::Mat& img_dst){
    cv::remap(img_source, img_dst, this->undist_map_x, this->undist_map_y, CV_INTER_LINEAR);
};

void CamLidarSyncAlign::saveLidarDataRingTime(const string& file_name){
    int n_pts = n_pts_lidar;

    std::ofstream output_file(file_name, std::ios::trunc);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);

    if(output_file.is_open()){
        output_file << "# .PCD v.7 - Point Cloud Data file format\n";
        output_file << "VERSION .7\n";
        output_file << "FIELDS x y z intensity ring time\n";
        output_file << "SIZE 4 4 4 4 2 4\n";
        output_file << "TYPE F F F F U F\n";
        output_file << "COUNT 1 1 1 1 1 1\n";
        output_file << "WIDTH " << n_pts << "\n";
        output_file << "HEIGHT 1\n";
        output_file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        output_file << "POINTS " << n_pts<< "\n";
        output_file << "DATA ascii\n";
        for(int i = 0; i < n_pts; i++){
            output_file << *(buf_lidar_x + i)<<" ";
            output_file << *(buf_lidar_y + i)<<" ";
            output_file << *(buf_lidar_z + i)<<" ";
            output_file << *(buf_lidar_intensity + i)<<" ";
            output_file << *(buf_lidar_ring + i)<<" ";
            output_file << *(buf_lidar_time + i)<<"\n";
        }
    }  
};

void CamLidarSyncAlign::saveSnapshot(){
    // save images
    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
    ++current_seq_;
    string file_name = save_dir_ + "/cam0/" + itos(current_seq_) + ".png";
	cv::imwrite(file_name, buf_img_, png_parameters);

    // save lidars
    file_name = save_dir_ + "/lidar0/" + itos(current_seq_) + ".pcd";
    saveLidarDataRingTime(file_name);

    // save association
    file_name = save_dir_ + "/association.txt";
    std::ofstream output_file(file_name, std::ios::app);
    output_file.precision(6);
    output_file.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(output_file.is_open()){
        output_file << buf_time_ << " ";
        output_file << "/cam0/" << current_seq_ << ".png ";
        output_file << "/lidar0/" << current_seq_ << ".pcd ";
        output_file << "\n";
    }
};

#endif
