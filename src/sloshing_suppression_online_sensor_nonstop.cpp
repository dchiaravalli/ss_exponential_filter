#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include "ss_exponential_filter/SS_filter.h"
#include "ss_exponential_filter/SS_parameter_evaluator.h"
#include "ss_exponential_filter/Tool_posefinder.h"
#include "ss_exponential_filter/ComauIK.h"

Eigen::Vector3d new_point,comau_EF,stabilizer;
Eigen::Vector3d mean_force_low,mean_torque_low,mean_force_max,mean_torque_max;
std::vector<double> comau_state,current_EF,comau_reference;
ss_exponential_filter::ComauIK *robot;
int mean_counter;
std::vector<bool> flags(10);

void ss_callback(const trajectory_msgs::JointTrajectoryPoint &traj);
void comau_callback(const sensor_msgs::JointState &state);
void geomagic_callback(const sensor_msgs::JointState &traj);
void sensor_callback(const geometry_msgs::Twist &sensor);
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags);			
void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out);
bool ikcheck(std::vector<double> &old,sensor_msgs::JointState &present);

void writeToIKFile(std::ofstream & traj_file, std::vector<double> & vect)
{
	for (int i = 0; i < vect.size(); i++)
	{
		traj_file << vect[i] << ";" ;
	}
	traj_file << std::endl;
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_online_sensor_nonstop");
    ros::NodeHandle n;
    
    //variable definition

	std::vector<double> check_msg;
	bool vicon,geomagic,initialize;														//start/stop the recording
	tf::TransformListener listener;														//listener to receive values from vicon
	tf::StampedTransform transform;														//transform storing current reference position														//current reference pose
    std::string mode;
	int sample_number,count;
    std::vector<Eigen::Vector3d> result;
    trajectory_msgs::JointTrajectoryPoint filtered_point;
	sensor_msgs::JointState state_msg;
    std::ofstream save_file,ikfile;
	std::stringstream filepath;
    double time1,time2,totaltime;
	KDL::Vector v;
    KDL::Frame target_frame;
	ros::Time tmsg;

    ss_exponential_filter::SS_filter filter(0.4809,0.002,-0.1014);
    ss_exponential_filter::SS_parameter_evaluator evaluator(0.002,0.094/2,1);
    ss_exponential_filter::Tool_posefinder tool(0.0275);

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


    //Publisher and Subscribers
    if (geomagic){
        ros::Subscriber trajectory_sub=n.subscribe("/Geomagic/end_effector_pose",1,geomagic_callback);
    }
    else if (!vicon)
        ros::Subscriber trajectory_sub=n.subscribe("suppress_sloshing_online",1,ss_callback);

	ros::Subscriber comau_sub=n.subscribe("/comau_smart_six/joint_states",1,comau_callback);
    ros::Subscriber sensor_sub=n.subscribe("ciao",1,sensor_callback);
    ros::Publisher comau_pub= n.advertise<sensor_msgs::JointState>("/comau_smart_six/joint_command",10);

	//variable initialisation
	comau_state.resize(6);
	comau_reference.resize(6);
	state_msg.position.resize(6);
	current_EF.resize(3);
	sample_number=filter.getFilterSampleNumber();
	count=sample_number;
    for (int i=0;i<3;i++){
        new_point[i]=0;
        mean_force_low[i]=0;
        mean_force_max[i]=0;
        mean_torque_low[i]=0;
        mean_torque_max[i]=0;
    }
    for (int i=0;i<10;i++){
        flags[i]=false;
    }
    flags[3]=true;
	robot = new ss_exponential_filter::ComauIK(127.5,0,153.7358);
	current_EF[0]=600;
	current_EF[1]=0;
	current_EF[2]=1400;
    mean_counter=0;

    //wait for transform to come up
    if (vicon){
        while (!listener.waitForTransform( "vicon", "pen",ros::Time(0), ros::Duration(10.0))){
		    std::cout<<"no transform published"<<std::endl;
	    }  
        std::cout<<"vicon system enabled"<<std::endl; 
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


	//start thread for recording data
	std::thread recorder(dataRecorder,std::ref(filepath),std::ref(flags));



    time2=ros::Time::now().toSec();
	ros::Rate r(500);
	while(ros::ok())
	{
        if (vicon)
        {
		//read new operator pose
            try
            {
                listener.lookupTransform( "vicon", "pen", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
			if (sqrt(pow(new_point[0]-transform.getOrigin().x(),2)+pow(new_point[1]-transform.getOrigin().y(),2)+pow(new_point[2]-transform.getOrigin().z(),2))<0.3){
				new_point[0]=transform.getOrigin().x();
				new_point[1]=transform.getOrigin().y();
				new_point[2]=transform.getOrigin().z();
        }
		}
        if (flags[4]){
            if (flags[6]){
                for (int i=0;i<3;i++)
                    comau_EF[i]=current_EF[i]/1000-new_point[i];
            }
            new_point=new_point+comau_EF;
            v=KDL::Vector(0,0,0);
			
    		target_frame.p=KDL::Vector(1000*new_point[0],1000*new_point[1],1000*new_point[2]);
    		target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*new_point[1],1000*new_point[0]));
    		robot->ik(target_frame,v,false,comau_reference);
			for (int i=0;i<6;i++){
				state_msg.position[i]=comau_reference[i];
			}
			//if (state_msg.position[4]<-1.8) state_msg.position[4]=-1.8;
			if(!ikcheck(check_msg,state_msg)){
				std::cout<<"big angles difference, stopping"<<std::endl;
				return 0;
			}
			tmsg=ros::Time::now();
			state_msg.header.stamp.sec=tmsg.sec;
			state_msg.header.stamp.nsec=tmsg.nsec;
            comau_pub.publish(state_msg);
        }
        if (flags[0]){
            time1=ros::Time::now().toSec();
		    //std::cout<<time2-time1<<std::endl;
		    time2=time1;
            if (flags[1]){
				for (int i=0;i<3;i++){
					comau_EF[i]=current_EF[i]/1000;
				}
                tool.setSensorRead(mean_force_max,mean_torque_max);
                evaluator.setLiquidMass(tool.getFluidMass(0.5));
                filter.setFilterParameters(evaluator.getFilterParameters());
                robot->setToolOffsets(1000*(tool.getToolPose(0.005,0.0335)+Eigen::Vector3d(0,0,evaluator.getFluidHeight())));
				filter.setFilterInitialState(comau_EF);
				comau_EF=new_point-comau_EF;
				filter.setFilterPositionOffset(comau_EF);
				robot->reset();
				flags[2]=false;
                flags[1]=false;
				count=0;
                save_file.open(filepath.str().c_str(),std::ofstream::out);
				ikfile.open("/home/davide/ros/sloshing_ws/files/verifica.csv",std::ofstream::out);
				result=filter.evaluateOnlineSingleFilterStep(new_point);
            	writeToFile(save_file,new_point,result[0],result[1]);
            	filtered_point=filter.EigenToJointTrajectorPoint(result[0],result[1]);
        		v=KDL::Vector(0,0,0);
            }
			else{
            result=filter.evaluateOnlineSingleFilterStep(new_point);
            writeToFile(save_file,new_point,result[0],result[1]);
            filtered_point=filter.EigenToJointTrajectorPoint(result[0],result[1]);
        	v=KDL::Vector(filtered_point.accelerations[0],filtered_point.accelerations[1],filtered_point.accelerations[2]);
			}
    		target_frame.p=KDL::Vector(1000*filtered_point.positions[0],1000*filtered_point.positions[1],1000*filtered_point.positions[2]);
    		//target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_pose.positions[1],1000*req.target_pose.positions[0]));
    		robot->ik(target_frame,v,true,comau_reference);
			 writeToIKFile(ikfile,comau_reference);
			for (int i=0;i<6;i++){
				state_msg.position[i]=comau_reference[i];
			}
			//if (state_msg.position[4]<-1.8) state_msg.position[4]=-1.8;
			if(!ikcheck(check_msg,state_msg)){
				std::cout<<"big angles difference, stopping"<<std::endl;
				return 0;
			}
			tmsg=ros::Time::now();
			state_msg.header.stamp.sec=tmsg.sec;
			state_msg.header.stamp.nsec=tmsg.nsec;
            comau_pub.publish(state_msg);
        }
        else
        {
			if (count<sample_number){
				if (count==0){
					stabilizer=new_point;
				}
				result=filter.evaluateOnlineSingleFilterStep(stabilizer);
				writeToFile(save_file,new_point,result[0],result[1]);
				filtered_point=filter.EigenToJointTrajectorPoint(result[0],result[1]);
				v=KDL::Vector(filtered_point.accelerations[0],filtered_point.accelerations[1],filtered_point.accelerations[2]);
				target_frame.p=KDL::Vector(1000*filtered_point.positions[0],1000*filtered_point.positions[1],1000*filtered_point.positions[2]);
				//target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*req.target_pose.positions[1],1000*req.target_pose.positions[0]));
				robot->ik(target_frame,v,true,comau_reference);
				 writeToIKFile(ikfile,comau_reference);
				for (int i=0;i<6;i++){
					state_msg.position[i]=comau_reference[i];
				}
				//if (state_msg.position[4]<-1.8) state_msg.position[4]=-1.8;
				if(!ikcheck(check_msg,state_msg)){
					std::cout<<"big angles difference, stopping"<<std::endl;
					return 0;
				}
				tmsg=ros::Time::now();
				state_msg.header.stamp.sec=tmsg.sec;
				state_msg.header.stamp.nsec=tmsg.nsec;
				comau_pub.publish(state_msg);
				count++;
				if (count==sample_number){
					flags[2]=true;
					save_file.close();
					ikfile.close();
					check_msg.clear();
				}

			}
        }


		ros::spinOnce();
		r.sleep();
				
	}
	// close file check
	if (save_file.is_open())
		save_file.close();
	return 0;
}

bool ikcheck(std::vector<double> &old,sensor_msgs::JointState &present){
	if (old.empty()){
		for (int i=0;i<6;i++){
			old.push_back(present.position[i]);
		}
		return true;
	}
	else{
		for(int i=0;i<6;i++){
			if(fabs(old[i]-present.position[i])>0.2) return false;
			old[i]=present.position[i];
		}
		return true;
	}
}


void comau_callback(const sensor_msgs::JointState &state){
	for (int i=0;i<6;i++){
		comau_state[i]=state.position[i];
	}
	current_EF=robot->fk(comau_state);
}
	

void ss_callback(const trajectory_msgs::JointTrajectoryPoint &traj){
    for (int i=0;i<3;i++){
        new_point[i]=traj.positions[i];
    }
}

void sensor_callback(const geometry_msgs::Twist &sensor){
    if (!flags[7]){
        if (!flags[9]){
            mean_force_low[0]+=sensor.linear.x;
            mean_force_low[1]+=sensor.linear.y;
            mean_force_low[2]+=sensor.linear.z;
            mean_torque_low[0]+=sensor.angular.x;
            mean_torque_low[1]+=sensor.angular.y;
            mean_torque_low[2]+=sensor.angular.z;
            mean_counter++;
            if (mean_counter=1000){
                mean_force_low=mean_force_low/mean_counter;
                mean_torque_low=mean_torque_low/mean_counter;
                mean_counter=0;
                flags[9]=true;
            }
        }
        else
        {
            if (fabs(sensor.linear.z-mean_force_low[2])>2){
                flags[8]=true;
            }
        }
        if (flags[8]){
            mean_force_max[0]+=sensor.linear.x;
            mean_force_max[1]+=sensor.linear.y;
            mean_force_max[2]+=sensor.linear.z;
            mean_torque_max[0]+=sensor.angular.x;
            mean_torque_max[1]+=sensor.angular.y;
            mean_torque_max[2]+=sensor.angular.z;
            mean_counter++;
            if (mean_counter=1000){
                mean_force_max=mean_force_max/mean_counter-mean_force_low;
                mean_torque_max=mean_torque_max/mean_counter-mean_torque_low;
                mean_counter=0;
                flags[7]=true;
                flags[8]=false;
            }
        }
    }

}
void geomagic_callback(const sensor_msgs::JointState &traj){
    for (int i=0;i<3;i++){
        new_point[i]=traj.position[i];
    }
}
	
void dataRecorder(std::stringstream &filepath,std::vector<bool> &flags)
{
	std::stringstream filename,vicontag;
	char again;
	std::string name;
		name.clear();
		//open the file for the trajectory
		filepath.str("");
		filename.str("");
		filepath<<"/home/davide/ros/sloshing_ws/files/online_trajectories/";
		std::cout<<"print the name of the file where the trajectory will be saved"<<std::endl;
		std::cout<<"it will be created in .../home/davide/ros/sloshing_ws/files/online_trajectories/"<<std::endl;
		std::cin >> name;
		filename << name;
		filepath<<filename.str();
        std::cout<<"waiting for the sensor to stabilize"<<std::endl;
        while (!flags[9]);
        std::cout<<"press enter to start"<<std::endl;
        std::cin.ignore();
        std::cin.get();
        flags[4] = true;
        flags[6]=true;
        std::cout << "press enter to stop " << std::endl;
        std::cin.ignore();
        std::cin.get();
        flags[4]=false;
        std::cout<<"waiting for sensor to receive payload and update filters"<<std::endl;
        while (!flags[7]);
		std::cout << "press enter to start" << std::endl;
		std::cin.ignore();
		std::cin.get();
		flags[1]=true;
		flags[0]=true;
		std::cout << "press enter to stop recording" << std::endl;
		std::cin.ignore();
		std::cin.get();
		flags[0]=false;
		std::cout<<"waiting for the trajectory to ultimate"<<std::endl; 
}

void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out)
{
	for (int i = 0; i < vect_in.size(); i++)
	{
		traj_file << vect_in[i] << ";" ;
	}
    for (int i = 0; i < vect_out.size(); i++)
	{
		traj_file << vect_out[i] << ";" ;
	}
    for (int i = 0; i < acc_out.size(); i++)
	{
		traj_file << acc_out[i] << ";" ;
	}
	traj_file << std::endl;
}
