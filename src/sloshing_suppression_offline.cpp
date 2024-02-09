#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <thread>
#include "ss_exponential_filter/SS_offline_control.h"

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_offline");
    ros::NodeHandle n;

    //publisher and subscriber topics definition
    std::vector<std::string> topics;
    topics.push_back("/comau_smart_six/joint_command");
    topics.push_back("/comau_smart_six/joint_states");
    topics.push_back("/home/davide/ros/sloshing_ws/files/offline_trajectories/p2pZ.csv");
    topics.push_back("/atift");

    //online control 
    ss_exponential_filter::SS_offline_control offline_control(n,topics,"recorder");

    //impose desired values for ss_filter
    std::vector<double> filter_parameters(3);
    filter_parameters[0] = 0.53;                                                          //filter period
    filter_parameters[1] = 0.002;                                                           //filter sampling time
    filter_parameters[2] = -0.4;                                                         //filter shape response factor
    offline_control.setSSFilterParameters(filter_parameters);

    //impose desired offset for robot_ik
    offline_control.setRobotIkOffsets(Eigen::Vector3d(127.5,0,-50));

    //set save file path
    offline_control.setSaveFilePath("/home/davide/ros/sloshing_ws/files/offline_tests/");

    //start input thread
    std::thread input = offline_control.inputThread();

    //enable control loop
    offline_control.spin();

    return 0;
}

