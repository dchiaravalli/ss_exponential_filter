#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <ss_exponential_filter/ComauIK.h>
#include <ss_exponential_filter/SS_parameter_evaluator.h>
#include <thread>
#include <kdl/frames_io.hpp>

std::vector<double> save_vector,pos_vector,joint_vector;

void write_to_file(std::ofstream & traj_file, std::vector<double> & vect)
{
	for (int i = 0; i < vect.size(); i++)
	{
		traj_file << vect[i] << ";" ;
	}
	traj_file << std::endl;
}

void write_all_to_file(std::ofstream & traj_file, std::vector<double> & vect,KDL::Frame &pos_vect,std::vector<double> &joint_vect)
{
	for (int i = 0; i < vect.size(); i++)
	{
		traj_file << vect[i] << ";" ;
	}
    for (int i = 0; i < 3; i++)
	{
		traj_file << pos_vect.p.data[i]/1000 << ";" ;
	}
    for(int i=0;i<9;i++)
    {
        traj_file << pos_vect.M.data[i] << ";" ;
    }
    for (int i = 0; i < joint_vect.size(); i++)
	{
		traj_file << joint_vect[i] << ";" ;
	}
	traj_file << std::endl;
}


void callback(const geometry_msgs::Twist &msg){
    save_vector[0]=msg.linear.x;
    save_vector[1]=msg.linear.y;
    save_vector[2]=msg.linear.z;
    save_vector[3]=msg.angular.x;
    save_vector[4]=msg.angular.y;
    save_vector[5]=msg.angular.z;
}

void posCallback(const sensor_msgs::JointState &msg){
    for(int i=0;i<6;i++){
        joint_vector[i]=msg.position[i];
    }

}


void commander(bool &go,bool &stop){
    	std::cout << "press enter to start" << std::endl;
		std::cin.ignore();
		std::cin.get();
		go = true;
        std::cout << "press enter to stop recording" << std::endl;
		std::cin.ignore();
		std::cin.get();
        stop = true;
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "force_sensor_recorder");
    ros::NodeHandle n;
	std::stringstream force_name,position_name;
    KDL::Frame pos_frame;
    save_vector.resize(6);
    joint_vector.resize(6);
    pos_vector.resize(3);
    std::fill(save_vector.begin(),save_vector.end(),0.0);
    std::fill(pos_vector.begin(),pos_vector.end(),0.0);
    std::fill(joint_vector.begin(),joint_vector.end(),0.0);
    bool go= false, stop = false,total = false;
    ss_exponential_filter::ComauIK robot(0,0,0);
    ss_exponential_filter::SS_parameter_evaluator evaluator(0.02);
    std::vector<double> properties;
    std::ofstream annotation_file;
    
    force_name.str();
    force_name << "/home/davide/ros/sloshing_ws/files/";


    if (argc>=2){
        std::string chapio = argv[1];
        force_name << "force_and_training/"<<chapio<<".csv";
        total=false;

    }
    else
        force_name << "force_sensor/new_reading.csv";

    if (argc>=4){
        evaluator.setSystemParameters(0.02,atof(argv[2]),atof(argv[3]));
        properties = evaluator.getLiquidParameters();
        annotation_file.open("/home/davide/ros/sloshing_ws/files/force_and_training/record_2/annotation.txt",std::ios::app);
        std::string str = argv[1];
        std::stringstream ss;
        ss << str <<" frequency: "<<properties[0]<<" damping: "<<properties[1]<<" radius: "<<atof(argv[2])<<" mass: "<<atof(argv[3]) ;
        annotation_file << ss.str();
        annotation_file << std::endl;
        annotation_file.close();
    }
        
    std::ofstream save_file;

    save_file.open(force_name.str().c_str());
    
    
    std::cout<<save_file.is_open()<<std::endl;    

    ros::Subscriber sub = n.subscribe("/atift",1,callback);
    ros::Subscriber pos_sub = n.subscribe("/comau_smart_six/joint_states",1,posCallback);

    std::thread recorder(commander, std::ref(go),std::ref(stop));
    
	ros::Rate r(500);
	while(ros::ok())
	{   
        if (go){
            if (total){
                pos_frame=robot.total_fk(joint_vector);
                write_all_to_file(save_file,save_vector,pos_frame,joint_vector);
            }
            else
                write_to_file(save_file,save_vector);
        }
        if (stop){
            save_file.close();
            ros::shutdown();
        }
		ros::spinOnce();
		r.sleep();		
	}
	return 0;
}
	
