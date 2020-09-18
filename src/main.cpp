#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"

#include "camlidar_sync_align.h"


using namespace std;
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
    CamLidarSyncAlign* cl = new CamLidarSyncAlign(nh);
    
    while(ros::ok()){
        ros::spinOnce();
    }

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
