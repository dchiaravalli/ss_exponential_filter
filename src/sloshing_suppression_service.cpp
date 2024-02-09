/*******************************************************************
* File: sloshing_suppression_service.cpp
*
* Authors: Davide Chiaravalli, Federico Califano
* 
* License: gpl
* 
* Description: this node controls the 



*********************************************************************/
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ss_exponential_filter/sloshing_trajectory.h>
#include <ss_exponential_filter/sloshing_point.h>
#include <unordered_map>
#include "ss_exponential_filter/SS_filter.h"

//global variable class for filter
ss_exponential_filter::SS_filter *filter,*filter_online;
std::unordered_map<std::string,ss_exponential_filter::SS_filter> map;

bool ss_service(ss_exponential_filter::sloshing_trajectory::Request &req, ss_exponential_filter::sloshing_trajectory::Response &res){
	std::vector<Eigen::Vector3d> trajectory;
	trajectory = filter->jointTrajectoryToEigen(req.traj);
	if (req.set_initial_pos.data==true)
	{
		Eigen::Vector3d initial_pose(req.initial_pose.positions[0],req.initial_pose.positions[1],req.initial_pose.positions[2]);
		filter->evaluateFilteredTrajectory(trajectory,initial_pose);
	}
	else filter->evaluateFilteredTrajectory(trajectory);
	res.filtered_traj = filter->EigenToJointTrajectory(filter->getFilteredTrajectory(),filter->getFilteredAcceleration());
	return true;
}

bool sso_service(ss_exponential_filter::sloshing_point::Request &req, ss_exponential_filter::sloshing_point::Response &res){
	Eigen::Vector3d point;
	std::vector<Eigen::Vector3d> result;
	point = filter_online->jointTrajectoryPointToEigen(req.new_point);
	if (req.new_trajectory.data==true)
		if (req.set_initial_pos.data==true)
		{
			Eigen::Vector3d initial_pose(req.initial_pose.positions[0],req.initial_pose.positions[1],req.initial_pose.positions[2]);
			map[req.username].setFilterInitialState(initial_pose);
			point=point-initial_pose;
			map[req.username].setFilterPositionOffset(point);
		}
		else map[req.username].setFilterInitialState(point);
	result = map[req.username].evaluateOnlineSingleFilterStep(point);
	res.filtered_point = filter_online->EigenToJointTrajectorPoint(result[0],result[1]);
	return true;
}

/**
* Main function
* @param argc 
* @param argv
*/
int main(int argc, char **argv)
{	
	//Ros initialisation
	ros::init(argc, argv, "sloshing_suppression_service");
  	ros::NodeHandle n;

	//Service definition
	ros::ServiceServer service = n.advertiseService("sloshing_suppression_service", ss_service);
	ros::ServiceServer service_online = n.advertiseService("sloshing_suppression_online_service", sso_service);

	//Class initialisation
	filter = new ss_exponential_filter::SS_filter();
	filter_online = new ss_exponential_filter::SS_filter();

	if (argc=2){
		filter->setFilterParameters(atof(argv[1]),atof(argv[2]));
	}
	else if (argc=3){
		filter->setFilterParameters(atof(argv[1]),atof(argv[2]),atof(argv[3]));
	}

	//ros spin rate
	ros::Rate r(3000);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}


}












