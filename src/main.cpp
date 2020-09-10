#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <string>
#include <sstream>

// keyboard input tool
#include "keyinput.h"
#include "improved_topic_logger.h"

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

    int n_cams   = -1;
    int n_lidars = -1;

    string dir;    
    ros::param::get("~n_cameras", n_cams);
    ros::param::get("~n_lidars",  n_lidars);
    ros::param::get("~directory", dir);
    
    
    stringstream ss1;
    ss1 << dir << currentDateTime() << "/";
    string save_dir = ss1.str();
    cout << "save directory:[" << save_dir << "]\n";


    // Ground control system class
    ImprovedTopicLogger* itl =
     new ImprovedTopicLogger(nh, n_cams, n_lidars, save_dir);

    // user input manual.
    string user_manual;
    stringstream ss;
    ss << "\n==============================================\n|" 
    << "  Press a key..." 
    << "\n|    c: continuous save mode"
    << "\n|    s: save one scene (but IMU always logged in background.)" 
    << "\n|    q: cease the program"
    << "\n| Select an input: \n";
    user_manual = ss.str();
    cout << user_manual;

    string cam_configure_manual;
    ss.clear();
    ss.flush();

    ss << "\n |\n L\n";

    int cnt = 0;
    while(ros::ok())
    {
        int c = getch(); // call my own non-blocking input function.
        ros::spinOnce();

        if(c == 's'){
            cout << "\n\n[Operation]: snapshot & save the current scene.\n";

            // send single query to all sensors.
            bool is_query_ok = itl->isDataReceivedAllSensors();
            
            // save all data.
            // timestamp of IMU should be larger than the other sensors.
            if(is_query_ok){
                itl->saveAllData();
            }
            else cout << "   fail to save...\n";
            cout << user_manual;         
        }
        else if(c == 'c'){
            cout << "\n\n[Operation]: continuous saving! press 'f' to stop streaming.\n\n";
            
            while(1){
                c = getch();
                ros::spinOnce();

                // send single query to all sensors.
                bool is_query_ok = itl->isDataReceivedAllSensors();

                // save all data
                if(is_query_ok){
                    itl->saveAllData();
                    itl->initializeAllFlags();

                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                }
                // else cout << "   fail to save...\n";
                if(c == 'f') break;
            }
            cout << user_manual;         
        }
        else if(c == 'f'){
            cout << "\n\n[Operation]: quit program\n\n";
            break;
        }

    }

    // delete
    delete itl;


ROS_INFO_STREAM("End of the program.\n");
return -1;
}
