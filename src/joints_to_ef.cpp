#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <Eigen/Dense>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <comau_control/IKServiceTotal.h>
#include <ss_exponential_filter/ComauIK.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl/frames_io.hpp>

comau_control::IKServiceTotal srv;
ros::ServiceClient service;

void writeAllToFile(std::ofstream &traj_file,trajectory_msgs::JointTrajectory &traj){
    for (int j=0;j<traj.points.size();j++){
        for (int i = 0; i < 6; i++)
        {
            traj_file << traj.points[j].positions[i] << ";" ;
        }
        traj_file << std::endl;
    }

}


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "joints_to_ef");
    ros::NodeHandle n;
    std::vector<std::vector<double>> traj;
    std::vector<double> point,angles;
    double dummy;
    point.resize(6);
    angles.resize(3);
    std::string name;
    bool pos=true;
    std::stringstream file,receive_file;
    KDL::Frame frame;
    std::ofstream print_file;
    ss_exponential_filter::ComauIK robot(0,0,0);
    int count=1;
    file.str(""); 
    receive_file.str("");
    file<<"/home/davide/ros/sloshing_ws/files/force_and_pose/";
    receive_file<<"/home/davide/ros/sloshing_ws/files/force_and_pose/";
    if (argc>1){
        name=argv[1];
    }
    file<<name<<".csv";
    receive_file<<name<<"_orient.csv";
    print_file.open(receive_file.str().c_str());
    std::ifstream save_file(file.str().c_str());
    save_file>>dummy;
    while(save_file.get()!=EOF){
        if (pos){
            save_file>>dummy;
            count = (count+1) %9;
            if (count==0){
                pos=false;
                continue;
            }
        }
        else{
            save_file>>point[count];
            count = (count+1) %6;
            if (count==0){
                traj.push_back(point);
                pos=true;;
            }
        }


    }
    std::cout<<"hope: "<<robot.total_fk(traj[0]).p.data[0]<<" "<<robot.total_fk(traj[0]).p.data[1]<<" "<<robot.total_fk(traj[0]).p.data[2]<<std::endl;
    for (int i=0;i<traj.size();i++){
        frame = robot.total_fk(traj[i]);
        for(int j=0;j<9;j++){
            print_file << frame.M.data[j] << ";" ;
        }
        frame.M.GetRPY(angles[0],angles[1],angles[2]);
        //if (angles[0]<0) angles[0]=angles[0]+2*M_PI;
        for (int j=0;j<3;j++){
            print_file << angles[j]<< ";" ;
        }
        print_file<<std::endl;
    }

    save_file.close();
    print_file.close();



	return 0;
}
	
	
