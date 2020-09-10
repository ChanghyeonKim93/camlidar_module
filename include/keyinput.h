#ifndef _KEYINPUT_H_
#define _KEYINPUT_H_
#include <ros/ros.h>
#include <iostream>
#include <termios.h>

// ref: https://answers.ros.org/question/63491/keyboard-key-pressed/

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0){
        ROS_ERROR("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0){
        ROS_ERROR("tcsetattr ICANON");
    }

    if(rv == -1){
        //ROS_ERROR("select");
    }
    else if(rv == 0){
        //ROS_INFO("no_key_pressed");
    }
    else{
        read(filedesc, &buff, len );
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0){
        ROS_ERROR ("tcsetattr ~ICANON");
    }
    return (buff);
}

#endif