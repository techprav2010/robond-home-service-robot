// ROS Libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // Motor Commands
#include "sensor_msgs/LaserScan.h" // Laser Data
#include "tf/transform_listener.h" // tf Tree

// C++ Libraries
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stack>
#include <string.h>

// ROS Publisher:Motor Commands, Subscriber:Laser Data, and Messages:Laser Messages & Motor Messages
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

const int STOP = 0;
const int FORWARD = 1;
const int BACKWARD =2;
const int TURN_LEFT = 3;
const int TURN_RIGHT = 4;
const int GO_LEFT = 5;
const int GO_RIGHT = 6;

std::string action_desc[7]  = {
        "[ROBOT] HALT! \n"

        , "[ROBOT] I'm going forward! \n"
        , "[ROBOT] I'm going back! \n"


        , "[ROBOT] I'm turning left! \n"
        , "[ROBOT] I'm turning right! \n"


        , "[ROBOT] I'm going left! \n"
        , "[ROBOT] I'm going right! \n"


};
float robot_move_types[7][2] = {
        //z, x
        { 0.0, 0.0}, //HALT

        { 0.0, 0.5}, //forward
        { 0.0, -0.5}, //back

        { 1.0, 0.0}, //turn left
        { -1.0, 0.0},//turn right

        { 0.25, 0.25}, //going left
        { -0.25, 0.25} //going right
};

// The robot_move function will be called by the laser_callback function each time a laser scan data is received
// This function will accept robot movements and actuate the robot's wheels accordingly
// Keep a low speed for better results
bool robot_move(const int move_type)
{
    if(move_type < 7){
        ROS_INFO("action %s", action_desc[move_type].c_str());
        motor_command.angular.z = robot_move_types[move_type][0];
        motor_command.linear.x =  robot_move_types[move_type][1];
    } else {
        ROS_INFO("[ROBOT_MOVE] Move type wrong! \n");
        return false;
    }

    //Publish motor commands to the robot and wait 10ms
    motor_command_publisher.publish(motor_command);
    usleep(10);
    return true;
}

bool following_wall = false;
bool thats_a_door = false;
bool crashed = false;

// The laser_callback function will be called each time a laser scan data is received
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Read and process laser scan values
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();
    float left_side = 0.0, right_side = 0.0;
    float range_min = laser_msg.range_max, range_max = laser_msg.range_min;
    int nan_count = 0;
    for (size_t i = 0; i < range_size; i++) {
        if (laser_ranges[i] < range_min) {
            range_min = laser_ranges[i];
        }

        if (std::isnan(laser_ranges[i])) {
            nan_count++;
        }
        if (i < range_size / 4) {
            if (laser_ranges[i] > range_max) {
                range_max = laser_ranges[i];
            }
        }

        if (i > range_size / 2) {
            left_side += laser_ranges[i];
        }
        else {
            right_side += laser_ranges[i];
        }
    }

    // Check if the robot has crashed into a wall
    if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
        crashed = true;
    }
    else {
        crashed = false;
    }

    // Assign movements to a robot that still did not crash
    if (!crashed) {

        if (range_min <= 0.5 && !thats_a_door) {
            following_wall = true;
            crashed = false;
            robot_move(STOP);

            if (left_side >= right_side) {
                robot_move(TURN_RIGHT);
            }
            else {
                robot_move(TURN_LEFT);
            }
        }  else {
            robot_move(STOP);
            if (following_wall) {
                if (range_max >= 2.0) {
                    thats_a_door = true;
                    following_wall = false;
                }
            }
            if (thats_a_door) {
                if (laser_ranges[0] <= 0.5) {
                    thats_a_door = false;
                } else {
                    robot_move(GO_RIGHT);
                }

            } else {
                robot_move(FORWARD);
            }
        }
    } else {
        robot_move(BACKWARD);
    }
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle n;
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100); //Twist size =100

    // sub laser
    laser_subscriber = n.subscribe("/scan", 1000, laser_callback);

    ros::Duration time_between_ros_wakeups(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        //sleep
        time_between_ros_wakeups.sleep();
    }

    return 0;
}