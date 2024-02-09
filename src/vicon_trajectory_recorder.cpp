#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ss_exponential_filter/ss_input_trajectory.h>


void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect);				//function that writes std::vector<double> coordinates to a file
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags,trajectory_msgs::JointTrajectory &traj,Eigen::Vector3d &initial_pose);								//function to define and open the receiving file

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "vicon_trajectory_recorder");
    ros::NodeHandle n;
    
    //variable definition
    std::ofstream save_file;															//stores the name of the recording files
	std::vector<bool> flags;																			//start/stop the recording
	tf::TransformListener listener;														//listener to receive values from vicon
	tf::StampedTransform transform;														//transform storing current reference position
	Eigen::Vector3d end_pose, initial_pose;														//current reference pose
	trajectory_msgs::JointTrajectoryPoint new_point;
	trajectory_msgs::JointTrajectory new_trajectory;
	std::stringstream filepath;
	ss_exponential_filter::ss_input_trajectory advanced_trajectory;


	//variable initialisation
	flags.resize(5);
	for (int i=0;i<4;i++){
		flags[i]=false;
	}
	flags[4]=true;
	new_point.positions.resize(3);
	advanced_trajectory.initial_pose.positions.resize(3);



	//ros publisher
	ros::Publisher trajectory_pub=n.advertise<trajectory_msgs::JointTrajectory>("default_suppress_sloshing_offline",1);
	ros::Publisher advanced_trajectory_pub=n.advertise<ss_exponential_filter::ss_input_trajectory>("advanced_suppress_sloshing_offline",1);
	ROS_INFO("iniziamo");
	//start thread for recording data
	std::thread recorder(dataRecorder, std::ref(filepath),std::ref(flags),std::ref(new_trajectory),std::ref(initial_pose));

	//wait for transform to come up
	while (!listener.waitForTransform( "vicon", "pen",ros::Time(0), ros::Duration(10.0))){
		std::cout<<"no transform published"<<std::endl;;
	}   
	ros::Rate r(1000);
	while(ros::ok())
	{
		//read new operator pose
		try
		{
			listener.lookupTransform( "vicon", "pen", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			//ROS_ERROR("%s",ex.what());
		}

		//record to file if enabled
		if(flags[0])
		{
			if (flags[1]){
				save_file.open(filepath.str().c_str(),std::ofstream::out);
				flags[1]=false;
			}
			std::cout<<save_file.is_open()<<std::endl;
			end_pose[0]=transform.getOrigin().x();
			end_pose[1]=transform.getOrigin().y();
			end_pose[2]=transform.getOrigin().z();
			writeToFile(save_file,end_pose);
			for (int i=0;i<3;i++)
			{
				new_point.positions[i]=end_pose[i];
			}
			new_trajectory.points.push_back(new_point);
		}
		else if (save_file.is_open())
			save_file.close();
		if(flags[2]){
			if (flags[3]){
				std::cout<<"advanced detected"<<std::endl;
				advanced_trajectory.set_initial_pos.data=true;
				advanced_trajectory.traj=new_trajectory;
				for (int i=0;i<3;i++){
					advanced_trajectory.initial_pose.positions[i]=initial_pose[i];
				}
				advanced_trajectory_pub.publish(advanced_trajectory);
				flags[3]=false;
				flags[2]=false;
			}
			else{
				trajectory_pub.publish(new_trajectory);
				flags[2]=false;
			}

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
	
	
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags ,trajectory_msgs::JointTrajectory &traj,Eigen::Vector3d &initial_pose)
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
		std::cout<<"print the name of the file where the trajectory will be saved"<<std::endl;
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
		traj.points.clear();
		flags[0]=true;
		flags[1]=true;
		std::cout << "press enter to stop recording" << std::endl;
		std::cin.ignore();
		std::cin.get();
		flags[0]=false;
		std::cout << "do you want to send the recorded trajectory to the ss_filter? (type 'y' to accept)" << std::endl;
		std::cin >> again;
		if (again=='y') flags[2]=true;
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

