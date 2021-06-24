///// CLIENT PROGRAM /////
#include <iostream>
#include <windows.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

using namespace std;

HANDLE sub_pipe;

void rightHapticsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    int temp1 = 0;
    int temp2 = 0;
    stringstream ss;
    for(int i=0;i<5;i++){
        temp1 = (int)(msg->position[i]);
        temp2 = (int)(msg->position[i]*10.0);
        temp2 = (int)(temp2 - 10.0*temp1);
        ss << temp1 << temp2;
    }
    
    string str = ss.str();
    const char* c = str.c_str();
    // cout << strlen(c)*sizeof(char) << endl;
    // cout << "Sending data to pipe..." << endl;
    DWORD numBytesWritten = 0;
    BOOL result2 = WriteFile(
        sub_pipe, // handle to our outbound pipe
        c, // data to send
        strlen(c) * sizeof(char), // length of data to send (bytes)
        &numBytesWritten, // will store actual amount of data sent
        NULL // not using overlapped IO
    );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manus_sub");
    ros::NodeHandle n;

    ros::Subscriber right_haptics_sub = n.subscribe("/right_haptics",1,rightHapticsCallback);

    cout << "Connecting to pipe..." << endl;
    // Open the named pipe
    // Most of these parameters aren't very relevant for pipes.
    sub_pipe = CreateFile(
        "\\\\.\\pipe\\sub_pipe",
        GENERIC_WRITE, // only need write access
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );
    if (sub_pipe == INVALID_HANDLE_VALUE) {
        cout << "Failed to connect to pipe." << endl;
        system("pause");
        return 1;
    }
    cout << "Writing data to pipe..." << endl;
    
    double timenow = ros::Time::now().toSec();

    cout << fixed << timenow << endl;

    ros::Rate rate(240);
    while(ros::ok){
        ros::spin();
    }
    // Close our pipe handle
    CloseHandle(sub_pipe);
    cout << "Done." << endl;
    system("pause");
    return 0;
}