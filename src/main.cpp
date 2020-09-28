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
    string param_directory = "/home/larrkchlaptop/catkin_ws/src/camlidar_module/params/bluefox_vlp16.yaml";    
    string save_directory;
    bool snapshot_onoff = false;
    ros::param::get("~parameter_directory", param_directory);
    ros::param::get("~snapshot_saving_onoff", snapshot_onoff);
    ros::param::get("~save_directory", save_directory);
    CamLidarSyncAlign* cl = new CamLidarSyncAlign(nh,param_directory, save_directory);
    

    // user input manual.
    
    string user_manual;
    stringstream ss;
    if(snapshot_onoff){
        ss << "\n==============================================\n|" 
        << "  Press a key..." 
        << "\n|    s: query & save one scene" 
        << "\n|  Select an input: \n";
        user_manual = ss.str();
        cout << user_manual;
        ss.clear();
        ss.flush();

        ss << "\n |\n L\n";
    }
    


    while(ros::ok()){
        ros::spinOnce();
        // no need 'spinOnce()' for all loop!!
        if(snapshot_onoff){
            int c = getch(); // call my own non-blocking input function
            if(c == 's') {
                cout << "\n\n[Operation]: snapshot & save the current scene.\n";

                // Save all data
                if(cl->getDataReady()){
                    // save and purge the current data!
                    cl->saveSnapshot();
                }
                else {
                    cout << "   fail to save...\n";
                }
                cout << user_manual;
            }
            else if((c != 0)) {
                cout << ": Un-identified command...\n";
                cout << user_manual;
            }
        }
    }
    

    ROS_INFO_STREAM("End of the program.\n");
    return -1;
}
