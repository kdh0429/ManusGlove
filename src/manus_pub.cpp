///// CLIENT PROGRAM /////
#include <iostream>
#include <windows.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "trajectory_msgs/JointTrajectory.h"

using namespace std;

sensor_msgs::JointState right_joints;
sensor_msgs::JointState left_joints;
ros::Publisher left_joint_pub;
ros::Publisher right_joint_pub;

bool left_ready = false;
bool right_ready = false;


sensor_msgs::JointState allegro_hand_joint_cmd;
trajectory_msgs::JointTrajectory qb_hand_joint_trajectory;
ros::Publisher allegro_pub;
ros::Publisher qb_pub;

double left_sum_mcp, left_sum_pip;

void right_joint_angle_pub(char buffer[127])
{
    int jointNum = buffer[3] - '0';
    double sign;
    double digits[5];
    double joint_value[4];
    
    //pipe
    for (int i = 0; i < 4; i++)
    {//convert char to digits
        sign = (buffer[5 + i * 8] == '-') ? -1.0 : 1.0;
        digits[0] = (double)(buffer[6 + i * 8] - '0');
        digits[1] = (double)(buffer[8 + i * 8] - '0');
        digits[2] = (double)(buffer[9 + i * 8] - '0');
        digits[3] = (double)(buffer[10 + i * 8] - '0');
        digits[4] = (double)(buffer[11 + i * 8] - '0');
        joint_value[i] = sign * (digits[0] + digits[1] * 0.1 + digits[2] * 0.01 + digits[3] * 0.001 + digits[4] * 0.0001);
    }

    //thumb : 0~3, index: 4~7, middle: 8~11, ring: 12~15, no pinky
    //allegro: index 0~3 , ... , ring: 8~11, thumb: 12~15
    if (jointNum != 4)
    {
        right_joints.position[jointNum*4] = joint_value[0];
        right_joints.position[jointNum*4+1] = joint_value[1];
        right_joints.position[jointNum*4+2] = joint_value[2];
        right_joints.position[jointNum*4+3] = joint_value[3];
    }

    //timestamp
    if (jointNum == 4)
    {
        if(right_ready == false) {
            right_ready = true;
            return;
        }
        right_joints.header.stamp = ros::Time::now();
        right_joint_pub.publish(right_joints);
    }

    allegro_hand_joint_cmd.position[0] = right_joints.position[4];
    allegro_hand_joint_cmd.position[1] = 0.097 + right_joints.position[5] * 1.5;
    allegro_hand_joint_cmd.position[2] = right_joints.position[6] * 1.5;
    allegro_hand_joint_cmd.position[3] = right_joints.position[7] * 1.5;

    allegro_hand_joint_cmd.position[4] = right_joints.position[8];
    allegro_hand_joint_cmd.position[5] = 0.396 + right_joints.position[9] *1.5;
    allegro_hand_joint_cmd.position[6] = right_joints.position[10] * 1.5;
    allegro_hand_joint_cmd.position[7] = right_joints.position[11] * 1.5;
    
    allegro_hand_joint_cmd.position[8] = right_joints.position[12];
    allegro_hand_joint_cmd.position[9] = right_joints.position[13] * 1.5;
    allegro_hand_joint_cmd.position[10] = right_joints.position[14] * 1.5;
    allegro_hand_joint_cmd.position[11] = right_joints.position[15] * 1.5;

    allegro_hand_joint_cmd.position[12] = right_joints.position[0]; 
    allegro_hand_joint_cmd.position[13] = right_joints.position[1]; 
    allegro_hand_joint_cmd.position[14] = right_joints.position[2] * 1.5;
    allegro_hand_joint_cmd.position[15] = right_joints.position[3] * 1.5;

    allegro_pub.publish(allegro_hand_joint_cmd);
}

void left_joint_angle_pub(char buffer[127])
{
    int jointNum = buffer[3] - '0';
    double sign;
    double digits[5];
    double joint_value[4];
    
    //pipe
    for (int i = 0; i < 4; i++)
    {//convert char to digits
        sign = (buffer[5 + i * 8] == '-') ? -1.0 : 1.0;
        digits[0] = (double)(buffer[6 + i * 8] - '0');
        digits[1] = (double)(buffer[8 + i * 8] - '0');
        digits[2] = (double)(buffer[9 + i * 8] - '0');
        digits[3] = (double)(buffer[10 + i * 8] - '0');
        digits[4] = (double)(buffer[11 + i * 8] - '0');
        joint_value[i] = sign * (digits[0] + digits[1] * 0.1 + digits[2] * 0.01 + digits[3] * 0.001 + digits[4] * 0.0001);
    }
    
    //cout <<digits[0] << " " << digits[1] << " " << digits[2] << " " << digits[3] << " " << digits[4] << endl;

    if (jointNum != 4)
    {
        left_joints.position[jointNum*4] = joint_value[0];
        left_joints.position[jointNum*4+1] = joint_value[1];
        left_joints.position[jointNum*4+2] = joint_value[2];
        left_joints.position[jointNum*4+3] = joint_value[3];
    }

    //timestamp
    if (jointNum == 4)
    {
        if(left_ready == false) {
            left_ready = true;
            return;
        }
        left_joints.header.stamp = ros::Time::now();
        left_joint_pub.publish(left_joints);
    }

    left_sum_mcp = left_joints.position[5]+left_joints.position[9]+left_joints.position[13];
    left_sum_mcp /= 2.0;
    left_sum_pip = left_joints.position[6]+left_joints.position[10]+left_joints.position[14];
    left_sum_pip /= 2.0;

    if (left_sum_mcp > 1.0)
        left_sum_mcp = 1.0;
    if (left_sum_pip > 1.0)
        left_sum_pip = 1.0;
        
    qb_hand_joint_trajectory.points[0].positions[0] = max(left_sum_mcp,left_sum_pip);
    ros::Duration traj_time(0.1);
    qb_hand_joint_trajectory.points[0].time_from_start = traj_time;
    qb_pub.publish(qb_hand_joint_trajectory);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manus_pub");
    ros::NodeHandle n;
    left_joint_pub = n.advertise<sensor_msgs::JointState>("left_glove_joints", 100);
    right_joint_pub = n.advertise<sensor_msgs::JointState>("right_glove_joints", 100);

    left_joints.position.resize(16);
    right_joints.position.resize(16);

    allegro_pub = n.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 100);	
	qb_pub = n.advertise<trajectory_msgs::JointTrajectory>("qbhand1/control/qbhand1_synergy_trajectory_controller/command", 1000);
    allegro_hand_joint_cmd.position.resize(16);

    qb_hand_joint_trajectory.joint_names.resize(1);
    qb_hand_joint_trajectory.joint_names[0] = "qbhand1_synergy_joint";
    qb_hand_joint_trajectory.points.resize(1);
    qb_hand_joint_trajectory.points[0].positions.resize(1);

    cout << "Connecting to pipe..." << endl;
    // Open the named pipe
    // Most of these parameters aren't very relevant for pipes.
    HANDLE pipe = CreateFile(
        "\\\\.\\pipe\\my_pipe",
        GENERIC_READ, // only need read access
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );
    if (pipe == INVALID_HANDLE_VALUE) {
        cout << "Failed to connect to pipe." << endl;
        // look up error code here using GetLastError()
        system("pause");
        return 1;
    }
    cout << "Reading data from pipe..." << endl;
    
    double timenow = ros::Time::now().toSec();

    cout.precision(17);
    cout << fixed << timenow << endl;
    // The read operation will block until there is data to read
    char buffer[127];
    DWORD numBytesRead = 0;
    BOOL result;
    ros::Rate rate(240);
    while(ros::ok){
        result = ReadFile(
            pipe,
            buffer, // the data from the pipe will be put here
            127 * sizeof(char), // number of bytes allocated
            &numBytesRead, // this will store number of bytes actually read
            NULL // not using overlapped IO
        );
        if (result) {
            ///////
            buffer[numBytesRead / sizeof(char)] = '\0'; // null terminate the string
            // cout << "Message: " << fixed << buffer << endl;
            //  +4 +0.2597 +0.0000 +0.0000 +0.0000 +1
            if(buffer[14 + 3 * 8] == '0') {
                right_joint_angle_pub(buffer);
            }
            else {
                left_joint_angle_pub(buffer);
            }
        } else {
            continue;
            // cout << "Failed to read data from the pipe." << endl;
        }
        rate.sleep();
    }
    // Close our pipe handle
    CloseHandle(pipe);
    cout << "Done." << endl;
    system("pause");
    return 0;
}