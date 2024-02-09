#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "ss_exponential_filter/SS_line_control.h"
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_online");
    ros::NodeHandle n;

    //get parameters from launch file
    double ss_period, sampling, obj_pose_x, obj_pose_y, obj_pose_z,period,displacement;
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/comau_smart_six/robot_ef",1);
    std::vector<double> fir_parameters(2);
    geometry_msgs::Pose robot_pose;
    bool compensation,reverse;
    n.getParam("fir_period_1",fir_parameters[0]);
    n.getParam("fir_period_2",fir_parameters[1]);
    n.getParam("period",ss_period);
    n.getParam("sampling",sampling);
    n.getParam("pos_x",obj_pose_x);
    n.getParam("pos_y",obj_pose_y);
    n.getParam("pos_z",obj_pose_z);
    n.getParam("compensation",compensation);
    n.getParam("reverse",reverse);
    n.getParam("displacement",displacement);


    //publisher and subscriber topics definition
    std::vector<std::string> topics;
    topics.push_back("/comau_smart_six/joint_command");
    topics.push_back("/comau_smart_six/joint_states");

    //online control 
    ss_exponential_filter::SS_line_control line_control(n,topics);



                                                          //filter sampling time

    //impose desired values for ss_filter
    std::vector<double> filter_parameters(3);
    filter_parameters[0] = ss_period;//0.4809;                                                          //filter period
    filter_parameters[1] = sampling;                                                           //filter sampling time
    filter_parameters[2] = -0.1014;                                                         //filter shape response factor

    
    //imposed trajectory points
    std::vector<Eigen::Vector3d> trajectory(2);
    trajectory[0] = Eigen::Vector3d(0,0,0);
    if (!reverse)
        trajectory[1] = Eigen::Vector3d(0,displacement,0);
    else
        trajectory[1] = Eigen::Vector3d(0,-displacement,0);
    //trajectory[2] = Eigen::Vector3d(0.4,0.2,0);
    //trajectory[3] = Eigen::Vector3d(0.2,0.1,0.1);
    //trajectory[2] = Eigen::Vector3d(-0.1,0.1,0);
    //trajectory[3] = Eigen::Vector3d(0,0,0);


    //object_pose
    Eigen::Vector3d object_pose(obj_pose_x,obj_pose_y,obj_pose_z);


    line_control.initialiseSystem(trajectory,object_pose,filter_parameters,fir_parameters,sampling,compensation);
    line_control.run(compensation);


    return 0;
}
