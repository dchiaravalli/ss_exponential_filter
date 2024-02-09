#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <ss_exponential_filter/sloshing_trajectory.h>
#include "ss_exponential_filter/SS_filter.h"
#include <ss_exponential_filter/ss_input_trajectory.h>

std::vector<Eigen::Vector3d> new_trajectory;
ss_exponential_filter::SS_filter *filter;
ros::ServiceClient service;
ros::Publisher trajectory_pub;
ss_exponential_filter::sloshing_trajectory srv;

void ss_advanced_callback(const ss_exponential_filter::ss_input_trajectory &traj){
    std::cout<<"advanced_trajectory_received "<< traj.traj.points[0].positions[0]<<std::endl;
    ss_exponential_filter::sloshing_trajectory srv;
    srv.request.traj=traj.traj;
    srv.request.set_initial_pos.data=traj.set_initial_pos.data;
    if (traj.set_initial_pos.data)
        srv.request.initial_pose=traj.initial_pose;
    if (service.call(srv)){
        trajectory_pub.publish(srv.response.filtered_traj);
        std::cout<<"trajectory filtered"<<std::endl;
    }
    else
        std::cout<<"service failed to respond"<<std::endl;
}

void ss_callback(const trajectory_msgs::JointTrajectory &traj){
    std::cout<<"trajectory_received "<< traj.points[0].positions[0]<<std::endl;
    ss_exponential_filter::sloshing_trajectory srv;
    srv.request.traj=traj;
    srv.request.set_initial_pos.data=false;
    if (service.call(srv)){
        trajectory_pub.publish(srv.response.filtered_traj);
        std::cout<<"trajectory filtered"<<std::endl;
    }
    else
        std::cout<<"service failed to respond"<<std::endl;
}



int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_offline_main");
    ros::NodeHandle n;
    
    //Publisher and Subscribers
    ros::Subscriber trajectory_sub_adv=n.subscribe("advanced_suppress_sloshing_offline",1,ss_advanced_callback);
    ros::Subscriber trajectory_sub=n.subscribe("default_suppress_sloshing_offline",1,ss_callback);
    trajectory_pub= n.advertise<trajectory_msgs::JointTrajectory>("ss_filtered_traj",10);
    service = n.serviceClient<ss_exponential_filter::sloshing_trajectory>("sloshing_suppression_service");

	ros::Rate r(1000);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
				
	}
	return 0;
}
	

