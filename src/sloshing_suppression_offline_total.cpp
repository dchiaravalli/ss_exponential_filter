#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ss_exponential_filter/sloshing_trajectory.h>
#include "ss_exponential_filter/SS_filter.h"

Eigen::Vector3d new_point;

void geomagic_callback(const sensor_msgs::JointState &traj);
void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect);				//function that writes std::vector<double> coordinates to a file
void writeAllToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj);
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags,Eigen::Vector3d &initial_pose);								//function to define and open the receiving file

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_offline_total");
    ros::NodeHandle n;
    
    //variable definition
    std::ofstream save_file,end_file;;															
	std::vector<bool> flags;																			
	tf::TransformListener listener;												
	tf::StampedTransform transform;														
	Eigen::Vector3d initial_pose;	
    std::vector<Eigen::Vector3d> eigen_traj;													
	std::stringstream filepath;
    ss_exponential_filter::sloshing_trajectory srv;
    bool vicon,geomagic;
    ss_exponential_filter::SS_filter filter;

	//variable initialisation
	flags.resize(5);
	for (int i=0;i<4;i++){
		flags[i]=false;
	}
    for (int i=0;i<3;i++){
        new_point[i]=0;
        initial_pose[i]=0;
    }
	flags[4]=true;
    eigen_traj.clear();
    srv.request.initial_pose.positions.resize(3);
    vicon=false;
    geomagic=false;
    if (argc>1) 
        if (std::string(argv[1])=="vicon")
        {
            vicon=true;
            std::cout<<"vicon system detected"<<std::endl;
        }

        else if (std::string(argv[1])=="geomagic")
            geomagic=true;

	


	//ros publisher
    if (geomagic){
        ros::Subscriber trajectory_sub=n.subscribe("/Geomagic/end_effector_pose",1,geomagic_callback);
    }
    ros::Publisher trajectory_pub= n.advertise<trajectory_msgs::JointTrajectory>("ss_filtered_traj",10);
    ros::ServiceClient service = n.serviceClient<ss_exponential_filter::sloshing_trajectory>("sloshing_suppression_service");

	//start thread for recording data
	std::thread recorder(dataRecorder, std::ref(filepath),std::ref(flags),std::ref(initial_pose));

	//wait for transform to come up
    if (vicon){
        while (!listener.waitForTransform( "vicon", "pen",ros::Time(0), ros::Duration(10.0))){
            std::cout<<"no transform published"<<std::endl;;
        }   
    
		try
		{
		listener.lookupTransform( "vicon", "pen", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		}
		new_point[0]=transform.getOrigin().x();
		new_point[1]=transform.getOrigin().y();
		new_point[2]=transform.getOrigin().z();
	}
	ros::Rate r(500);
	while(ros::ok())
	{
		//read new operator pose
        if (vicon)
        {
            try
            {
                listener.lookupTransform( "vicon", "pen", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
			//std::cout<<sqrt(pow(new_point[0]-transform.getOrigin().x(),2)+pow(new_point[1]-transform.getOrigin().y(),2)+pow(new_point[2]-transform.getOrigin().z(),2))<<std::endl;
			if (sqrt(pow(new_point[0]-transform.getOrigin().x(),2)+pow(new_point[1]-transform.getOrigin().y(),2)+pow(new_point[2]-transform.getOrigin().z(),2))<0.3){
				new_point[0]=transform.getOrigin().x();
				new_point[1]=transform.getOrigin().y();
				new_point[2]=transform.getOrigin().z();
			}

        }


		//record to file if enabled
		if(flags[0])
		{
			if (flags[1]){
                eigen_traj.clear();
				save_file.open(filepath.str().c_str(),std::ofstream::out);
				flags[1]=false;
			}

			writeToFile(save_file,new_point);
			eigen_traj.push_back(new_point);

		}
		else if (save_file.is_open())
			save_file.close();
		if(flags[2]){
			if (flags[3])
            {
				std::cout<<"advanced detected"<<std::endl;
				srv.request.traj=filter.EigenToJointTrajectoryPos(eigen_traj);
                srv.request.set_initial_pos.data=true;
				for (int i=0;i<3;i++)
                {
					srv.request.initial_pose.positions[i]=initial_pose[i];
				}

				flags[3]=false;

			}
			else
            {
                srv.request.traj=filter.EigenToJointTrajectoryPos(eigen_traj);
                std::cout<<"trajectory of dim: "<<srv.request.traj.points.size()<<std::endl;
                srv.request.set_initial_pos.data=false;

			}
            if (service.call(srv))
            {
                trajectory_pub.publish(srv.response.filtered_traj);
                end_file.open(filepath.str().c_str(),std::ofstream::out);
                std::cout<<"trajectory of dim: "<<srv.response.filtered_traj.points.size()<<std::endl;
                writeAllToFile(end_file,srv.response.filtered_traj);
                std::cout<<"trajectory filtered"<<std::endl;
            }
            else
            std::cout<<"service failed to respond"<<std::endl;
            flags[2]=false;
		}
		if(!flags[4])
		{
			if (save_file.is_open())
				save_file.close();
			ros::shutdown();
		}
		ros::spinOnce();
		r.sleep();		
	}
	if (save_file.is_open())
		save_file.close();
	return 0;
}
	
	
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags ,Eigen::Vector3d &initial_pose)
{
	std::stringstream filename,vicontag;
	char again;
	std::string name;
	while(flags[4]){
		name.clear();
		//open the file for the trajectory
		filepath.str("");
		filename.str("");
		vicontag<<"vicon_x;vicon_y;vicon_z;";
		filepath<<"/home/davide/ros/sloshing_ws/files/";
		std::cout<<"type the name of the file where the trajectory will be saved"<<std::endl;
		std::cout<<"it will be created in .../home/davide/ros/sloshing_ws/files/"<<std::endl;
		std::cin >> name;
		filename << name;
		filepath<<filename.str();
		//save_file<<vicontag.str()<<std::endl;
		std::cout<<"do you want to define a different initial pose? (type 'y' to accept)"<<std::endl;
		std::cin >> again;
		if (again=='y') {
			flags[3]=false;
			while(!flags[3]){
				std::cout<< "provide the starting x pose"<<std::endl;
				std::cin >>initial_pose[0];
				std::cout << "do you confirm the new x pose in: "<<initial_pose[0]<<" (type 'y' to accept)"<<std::endl;
				std::cin>>again;
				if(again=='y') flags[3]=true;
			}
			flags[3]=false;
			while(!flags[3]){
				std::cout<< "provide the starting y pose"<<std::endl;
				std::cin >>initial_pose[1];
				std::cout << "do you confirm the new y pose in: "<<initial_pose[1]<<" (type 'y' to accept)"<<std::endl;
				std::cin>>again;
				if(again=='y') flags[3]=true;
			}
			flags[3]=false;
			while(!flags[3]){
				std::cout<< "provide the starting z pose"<<std::endl;
				std::cin >>initial_pose[2];
				std::cout << "do you confirm the new z pose in: "<<initial_pose[2]<<" (type 'y' to accept)"<<std::endl;
				std::cin>>again;
				if(again=='y') flags[3]=true;
			}
		}
		else
		{		std::cin.ignore();
		std::cin.get();

		}
		std::cout << "press enter to start" << std::endl;
		std::cin.ignore();
		std::cin.get();
		flags[0]=true;
		flags[1]=true;
		std::cout << "press enter to stop recording" << std::endl;
		std::cin.ignore();
		std::cin.get();
		flags[0]=false;
		std::cout << "do you want to filter the recorded trajectory ? (type 'y' to accept)" << std::endl;
		std::cin >> again;
		if (again=='y') 
        {
            filepath.str("");
            filepath<<"/home/davide/ros/sloshing_ws/files/";
            std::cout<<"type the name of the file where the filtered trajectory will be saved"<<std::endl;
            std::cout<<"it will be created in .../home/davide/ros/sloshing_ws/files/"<<std::endl;
            std::cin >> name;
            filepath<<name;
            flags[2]=true;
        }
        while(flags[2]);
		std::cout << "do you want to record another trajectory? (type 'y' to accept)" << std::endl;
		std::cin >> again;
		if (again!='y') flags[4] = false;
	}
}

void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect)
{
	for (int i = 0; i < vect.size(); i++)
	{
		traj_file << vect[i] << ";" ;
	}
	traj_file << std::endl;
}

void geomagic_callback(const sensor_msgs::JointState& msg)
{
    new_point[0]=msg.position[0];
    new_point[1]=-msg.position[2];
    new_point[2]=msg.position[1];
}

void writeAllToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 3; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        for (int i = 0; i < 3; i++)
        {
            traj_file << traj.points[j].accelerations[i] << ";" ;
        }
        traj_file << std::endl;
    }

}