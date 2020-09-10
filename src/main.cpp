#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"

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

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cam_module");
    ros::NodeHandle nh("~");

    while(1){
	}

	ROS_INFO_STREAM("End of the program.\n");
	return -1;
}
