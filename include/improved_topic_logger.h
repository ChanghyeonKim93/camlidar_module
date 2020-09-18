#ifndef _ImprovedTopicLogger_H_
#define _ImprovedTopicLogger_H_
#include <stdlib.h>
#include <iostream>
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

// for subscribe
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Int32.h> // command msg
#include <sensor_msgs/TimeReference.h> // arduino time
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// custom msgs
#include "improved_topic_logger/imu_serial.h" // dedicated msgs for imu mpu6050

using namespace std;


class ImprovedTopicLogger{
public:
    ImprovedTopicLogger(ros::NodeHandle& nh, int n_cams, int n_lidars, const string& save_dir);
    ~ImprovedTopicLogger();
    void streamingMode();
    void pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void saveLidarDataRingTime(const std::string& file_name, const int& id);
    void saveAllData();

    // pointer to data
    cv::Mat& getBufImage(const int& id){ return *(buf_imgs_+id);};
    pcl::PointCloud<pcl::PointXYZI>::Ptr& getBufLidar(const int& id){return *(buf_lidars_+id);};

    inline int getNumCams(){return n_cams_;};
    inline int getNumLidars(){return n_lidars_;};
    
    bool isDataReceivedAllSensors();
    void initializeAllFlags();
private: // private methods
    void callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id);
    void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id);
    void callbackMcu(const improved_topic_logger::imu_serial::ConstPtr& msg_imu_serial);
    void serialParser(const std::string& serial_data, float* acc_f_, float* gyro_f_, float& temp_f);

private:
    // node handler
    ros::NodeHandle nh_;
    
    // subscribers
    image_transport::ImageTransport it_;
    vector<image_transport::Subscriber> subs_imgs_; // from mvBlueCOUGAR-X cameras
    ros::Subscriber sub_mcu_; // from arduino.
    vector<ros::Subscriber> subs_lidars_; // from Velodyne lidars

    // topic names
    vector<string> topicnames_imgs_;
    vector<string> topicnames_lidars_;
    string topicname_mcu_;

    // file names
    string filename_imu;
    string filename_association;

    // file descriptor
    ofstream fid_imu, fid_association;

    // state variables
    int n_cams_; // numbering rule(cams) 0,1) cabin left,right, 2,3) boom frontal,rear
    int n_lidars_;// numbering rule(lidars)- 0) cabin, 1) boom

    // transmittion flags.
    bool* flag_imgs_; // array. 'true' when image data is received.
    bool* flag_lidars_; // array. 'true' when lidar data is received.
    bool flag_mcu_; // scalar. 'true' when arduino data is received. 

    // data container (buffer)
    cv::Mat* buf_imgs_; // Images from mvBlueCOUGAR-X cameras.
    double imu_time_; // imu time!
    double buf_time_; // triggered time stamp from Arduino. (second)
    pcl::PointCloud<pcl::PointXYZI>::Ptr* buf_lidars_; // point clouds (w/ intensity) from Velodyne VLP16 
    
    vector<float*> buf_lidars_x;
    vector<float*> buf_lidars_y;
    vector<float*> buf_lidars_z;
    vector<float*> buf_lidars_intensity;
    vector<unsigned short*> buf_lidars_ring;
    vector<float*> buf_lidars_time;
    vector<int> buf_lidars_npoints;

    // current imu data
    int acc_i[3], gyro_i[3];
    int temp_i;

    float acc_f[3], gyro_f[3], temp_f;

    string save_dir_;
    int current_seq_;
    int last_seq_img_;
    int last_seq_lidar_;
};


ImprovedTopicLogger::ImprovedTopicLogger(ros::NodeHandle& nh,
    int n_cams, int n_lidars, const string& save_dir)
: nh_(nh), it_(nh_), n_cams_(n_cams), n_lidars_(n_lidars),save_dir_(save_dir)
{
    // default.
    flag_lidars_ = nullptr;
    flag_imgs_   = nullptr;
    buf_imgs_    = nullptr;

    // initialize image container & subscribers.
    if(n_cams_ > 0){
        buf_imgs_  = new cv::Mat[n_cams_];
        flag_imgs_ = new bool[n_cams_];
        for(int i = 0; i < n_cams_; i++) {
            flag_imgs_[i] = false;
            string name_temp = "/" + itos(i) + "/image_raw";
            topicnames_imgs_.push_back(name_temp);
            subs_imgs_.push_back(it_.subscribe(topicnames_imgs_[i], 1, boost::bind(&ImprovedTopicLogger::callbackImage, this, _1, i)));
        }
    }
    else {
        buf_imgs_  = nullptr;
        flag_imgs_ = nullptr;
    }

    // initialize lidar container & subscribers.
    if(n_lidars_ > 0){
        buf_lidars_ = new pcl::PointCloud<pcl::PointXYZI>::Ptr[n_lidars_];
        flag_lidars_ = new bool[n_lidars_];
        for(int i = 0; i < n_lidars_; i++){
            flag_lidars_[i] = false;
            *(buf_lidars_ + i) = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
	        buf_lidars_x.push_back(new float[100000]);
	        buf_lidars_y.push_back(new float[100000]);
	        buf_lidars_z.push_back(new float[100000]);
	        buf_lidars_intensity.push_back(new float[100000]);
	        buf_lidars_ring.push_back(new unsigned short[100000]);
	        buf_lidars_time.push_back(new float[100000]);
	        buf_lidars_npoints.push_back(0);

            string name_temp = "/lidar" + itos(i) + "/velodyne_points";
            topicnames_lidars_.push_back(name_temp);
            subs_lidars_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(topicnames_lidars_[i], 1, boost::bind(&ImprovedTopicLogger::callbackLidar, this, _1, i)));
        }
    }
    else{
        buf_lidars_  = nullptr;
        flag_lidars_ = nullptr;
    }
    
    // initialize arduino container & subscriber.
    flag_mcu_ = false;
    imu_time_ = -1.0;
    buf_time_ = -1.0;
    sub_mcu_ = nh_.subscribe("/mcu/mpu6050", 30, &ImprovedTopicLogger::callbackMcu, this);


    // generate save folder
    std::string folder_create_command;
    folder_create_command = "sudo rm -rf " + save_dir_;
	system(folder_create_command.c_str());
    folder_create_command = "mkdir " + save_dir_;
	system(folder_create_command.c_str());

    // make image saving directories
    for(int i = 0; i< n_cams_; i++){
        folder_create_command = "mkdir " + save_dir_ + "cam" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // make lidar data saving directories
    for(int i = 0; i < n_lidars_; i++){
        folder_create_command = "mkdir " + save_dir_ + "lidar" + itos(i) + "/";
	    system(folder_create_command.c_str());
    }

    // save association
    filename_association = save_dir_ + "/association.txt";
    fid_association = std::ofstream(filename_association, std::ios::trunc);
    fid_association.precision(6);
    fid_association.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(fid_association.is_open()){
        fid_association << "#seq time_us ";
        for(int i = 0; i < n_cams_; i++) fid_association << "cam" << i << " ";
        fid_association << "exposure_us gain_dB ";
        for(int i = 0; i < n_lidars_; i++) fid_association << "lidar" << i <<" ";
        fid_association << "\n";
    }

    // save IMU data
    filename_imu = save_dir_ + "/imu.txt";
    fid_imu = std::ofstream(filename_imu, std::ios::trunc);
    fid_imu.precision(8);
    fid_imu.setf(std::ios_base::fixed, std::ios_base::floatfield);
    if(fid_imu.is_open()){
        fid_imu << "#seq trigger time[us] ax[m/s2] ay[m/s2] az[m/s2] gx[rad/s] gy[rad/s] gz[rad/s] temperature[celcius]\n";
    }
};

ImprovedTopicLogger::~ImprovedTopicLogger() {
    // ! all allocation needs to be freed.
    if( buf_imgs_ != nullptr ) delete[] buf_imgs_;
    if( flag_imgs_ != nullptr ) delete[] flag_imgs_;

    if( buf_lidars_ != nullptr) delete[] buf_lidars_;
    if( flag_lidars_ != nullptr) delete[] flag_lidars_;
    for(int i = 0; i < n_lidars_; i++){
	delete[] buf_lidars_x[i];
	delete[] buf_lidars_y[i];
	delete[] buf_lidars_z[i];
	delete[] buf_lidars_intensity[i];
	delete[] buf_lidars_ring[i];
	delete[] buf_lidars_time[i];
    }
};

void ImprovedTopicLogger::streamingMode(){
    cout << "20 Hz (forced) streaming mode\n";
    // initialize all flags
    initializeAllFlags();
    
    // (timeout) Wait for obtaining and transmitting all sensor data. 
    // Considering exposure time and lidar gathering time, set 50 ms
    ros::spinOnce();
    ros::Duration(0.05).sleep();
};


bool ImprovedTopicLogger::isDataReceivedAllSensors()
{       
    // Check whether all data is received.
    bool transmit_success = true;
    if(flag_imgs_ != nullptr){
        for(int i = 0; i < n_cams_; i++){
            transmit_success = transmit_success & flag_imgs_[i];
            cout << "rcvd img[" << i<<"]\n";
	    }
    }
    if(flag_lidars_ != nullptr){
        for(int i = 0; i < n_lidars_; i++){
            transmit_success = transmit_success & flag_lidars_[i];
            cout << "rcvd lidar[" << i<<"]\n";
	    }
    }

    transmit_success = transmit_success & flag_mcu_;
    if(flag_mcu_) cout << "rcvd mcu\n";

    if(transmit_success) cout << " Transmission succeeds!\n";
    else cout << "Fail to transmit! Please retry...\n";

    return transmit_success;
};

void ImprovedTopicLogger::initializeAllFlags(){
    for(int i = 0; i < n_cams_; i++) flag_imgs_[i] = false;
    for(int i = 0; i < n_lidars_; i++){
        flag_lidars_[i] = false;
	    buf_lidars_npoints[i] = 0;
    }
    flag_mcu_ = false;
};

void ImprovedTopicLogger::callbackImage(const sensor_msgs::ImageConstPtr& msg, const int& id){
    cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	*(buf_imgs_ + id) = cv_ptr->image;

    cout << "  GCS get! [" << id << "] image.\n";
    flag_imgs_[id] = true;
};

void ImprovedTopicLogger::pointcloud2tobuffers(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
    // get width and height of 2D point cloud data
    buf_lidars_npoints[id] = msg_lidar->width;
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

       memcpy(buf_lidars_x[id]+i, &msg_lidar->data[arrayPosX], sizeof(float));
       memcpy(buf_lidars_y[id]+i, &msg_lidar->data[arrayPosY], sizeof(float));
       memcpy(buf_lidars_z[id]+i, &msg_lidar->data[arrayPosZ], sizeof(float));
       memcpy(buf_lidars_intensity[id]+i, &msg_lidar->data[ind_intensity], sizeof(float));
       memcpy(buf_lidars_ring[id]+i, &msg_lidar->data[ind_ring], sizeof(unsigned short));
       memcpy(buf_lidars_time[id]+i, &msg_lidar->data[ind_time], sizeof(float));
       //cout << "xyz intensity ring time: "<<*(buf_lidars_x[id]+i)<<","<<*(buf_lidars_y[id]+i)<<","<<*(buf_lidars_z[id]+i)
       //<<","<<*(buf_lidars_intensity[id]+i)<<","<<*(buf_lidars_ring[id]+i)<<","<<*(buf_lidars_time[id]+i)<<endl;
    }
}

void ImprovedTopicLogger::callbackLidar(const sensor_msgs::PointCloud2ConstPtr& msg_lidar, const int& id){
	pointcloud2tobuffers(msg_lidar,  id);
    msg_lidar->header.stamp; // timestamp

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *msg_lidar;

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = *(buf_lidars_ + id);
    temp->clear();
    pcl::fromROSMsg(output, *temp);

    int n_pts = temp->points.size();
    cout <<"n_pts lidar: " <<n_pts<<endl;
    flag_lidars_[id] = true;
};

void ImprovedTopicLogger::callbackMcu(const improved_topic_logger::imu_serial::ConstPtr& msg_imu_serial){
    imu_time_ = (double)msg_imu_serial->stamp.sec + (double)msg_imu_serial->stamp.nsec/(double)1000000.0;
    current_seq_ = msg_imu_serial->seq;

    char* serial_str = (char*)msg_imu_serial->data.c_str();
    char* tok1 = strtok(serial_str,","); // first : identifier
    tok1 = strtok(NULL,","); // 2: ax
    acc_i[0] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 3: ay
    acc_i[1] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 4: az
    acc_i[2] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 5: gx
    gyro_i[0] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 6: gy
    gyro_i[1] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 7: gz
    gyro_i[2] = std::stoi(tok1)-32768;
    tok1 = strtok(NULL,","); // 8: temperature
    temp_i = std::stoi(tok1)-32768;


    acc_f[0] = (float)acc_i[0]*0.0005985443115234f; // 9.80655 [m/s^2] /16384 (LSB);
    acc_f[1] = (float)acc_i[1]*0.0005985443115234f;
    acc_f[2] = (float)acc_i[2]*0.0005985443115234f;
    gyro_f[0] = (float)gyro_i[0]*0.00053211257682f; // 1 [deg] / 32.8 (LSB) * pi [rad] / 180 [deg]
    gyro_f[1] = (float)gyro_i[1]*0.00053211257682f;
    gyro_f[2] = (float)gyro_i[2]*0.00053211257682f;
    temp_f = (float)temp_i/340.0f+36.53f;

    int trg = 0;
    if(msg_imu_serial->flag_trigger){
        buf_time_ = imu_time_;
        last_seq_img_ = current_seq_;
        trg = 1;
    }
    // save imu data
    if(fid_imu.is_open()){
        fid_imu << current_seq_ << " " << trg << " "
        << imu_time_ << " " 
        << acc_f[0] <<" " << acc_f[1]<< " " << acc_f[2] << " "
        << gyro_f[0] <<" " << gyro_f[1]<< " " << gyro_f[2] << " "
        << temp_f << "\n";
    }
    
    cout << "  GCS get! [" << buf_time_ <<"] time ref."<<" seg: " << current_seq_ << "\n";
    flag_mcu_ = true;
};

void ImprovedTopicLogger::serialParser(const std::string& serial_data, float* acc_f_, float* gyro_f_, float& temp_f){

};


void ImprovedTopicLogger::saveLidarDataRingTime(const std::string& file_name, const int& id){
    int n_pts = buf_lidars_npoints[id];

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
            output_file << *(buf_lidars_x[id] + i)<<" ";
            output_file << *(buf_lidars_y[id] + i)<<" ";
            output_file << *(buf_lidars_z[id] + i)<<" ";
            output_file << *(buf_lidars_intensity[id] + i)<<" ";
            output_file << *(buf_lidars_ring[id] + i)<<" ";
            output_file << *(buf_lidars_time[id] + i)<<"\n";
        }
    }  
};

void ImprovedTopicLogger::saveAllData(){
    // save images
    bool static png_param_on = false;
	vector<int> static png_parameters;
	if (png_param_on == false)
	{
		png_parameters.push_back(CV_IMWRITE_PNG_COMPRESSION); // We save with no compression for faster processing
		png_parameters.push_back(0);
		png_param_on = true;
	}
    for(int id = 0; id < n_cams_; id++){
        string file_name = save_dir_ + "/cam" + itos(id) + "/" + itos(last_seq_img_) + ".png";
	    cv::imwrite(file_name, *(buf_imgs_ + id), png_parameters);
    }

    // save lidars
    for(int id = 0; id < n_lidars_; id++){
        string file_name = save_dir_ + "/lidar" + itos(id) + "/" + itos(last_seq_lidar_) + ".pcd";
        saveLidarDataRingTime(file_name, id);
    }

    // save association
    if(fid_association.is_open()){
        fid_association << last_seq_img_ << " ";
        fid_association << buf_time_ << " ";
        for(int i = 0; i < n_cams_; i++) fid_association << "/cam" << i << "/" << last_seq_img_ << ".png ";
        fid_association << 10000 << " " << 0 << " ";

        for(int i = 0; i < n_lidars_; i++) fid_association << "/lidar" << i << "/" << last_seq_lidar_ << ".pcd ";
        fid_association << "\n";
    }
};



#endif
