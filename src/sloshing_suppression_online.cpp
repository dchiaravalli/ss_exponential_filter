#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <thread>
#include "ss_exponential_filter/SS_online_control.h"

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_online");
    ros::NodeHandle n;

    //publisher and subscriber topics definition
    std::vector<std::string> topics;
    topics.push_back("/comau_smart_six/joint_command");
    topics.push_back("/comau_smart_six/joint_states");
    topics.push_back("/vicon_interpolation");
    topics.push_back("/atift");

    //online control 
    ss_exponential_filter::SS_online_control online_control(n,topics,"vicon_interpolated");

    //impose desired values for ss_filter
    std::vector<double> filter_parameters(3);
    filter_parameters[0] = 0.4809;                                                          //filter period
    filter_parameters[1] = 0.002;                                                           //filter sampling time
    filter_parameters[2] = -0.1014;                                                         //filter shape response factor
    online_control.setSSFilterParameters(filter_parameters);

    //impose desired offset for robot_ik
    online_control.setRobotIkOffsets(Eigen::Vector3d(132,0,-70));

    //set save file path
    online_control.setSaveFilePath("/home/davide/ros/sloshing_ws/files/online_trajectories/");

    //wait for vicon message on tf
    //std::cout<<"waiting for vicon system to receive first input"<<std::endl;
    //online_control.checkVicon("vicon","pen2");

    //start input thread
    std::thread input = online_control.inputThread();

    //enable control loop
    online_control.spin("vicon","pen2");

    return 0;
}

