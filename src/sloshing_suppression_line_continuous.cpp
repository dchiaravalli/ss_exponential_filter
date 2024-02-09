#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include "ss_exponential_filter/SS_line_control.h"
#include "ss_exponential_filter/motion_measure.h"
#include <ss_exponential_filter/LocateObjectAction.h>
#include <actionlib/client/simple_action_client.h>

void write_to_file(std::ofstream & traj_file, std_msgs::Float64MultiArray & vect)
{
	for (int i = 0; i < vect.data.size(); i++)
	{
		traj_file << vect.data[i] << ";" ;
	}
	traj_file << std::endl;
}


int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "sloshing_suppression_online");
    ros::NodeHandle n;

    //get parameters from launch file
    ros::Publisher result_pub =n.advertise<ss_exponential_filter::motion_measure>("motion_result",1);
    double ss_period, sampling, obj_pose_x, obj_pose_y, obj_pose_z,period,displacement;
    actionlib::SimpleActionClient<ss_exponential_filter::LocateObjectAction> ac("locate_objects", true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer();

    //goal message
    ss_exponential_filter::LocateObjectGoal goal;
    std_msgs::Float64MultiArray result;


    std::string filename;

    int loop_number;
    std::vector<double> fir_parameters(2);
    bool compensation,reverse;
    n.getParam("fir_period_1",fir_parameters[0]);
    n.getParam("fir_period_2",fir_parameters[1]);
    n.getParam("period",ss_period);
    n.getParam("sampling",sampling);
    n.getParam("compensation",compensation);
    n.getParam("displacement",displacement);
    n.getParam("loops",loop_number);
    n.getParam("savefile",filename);

    //save file
    std::ofstream save_file;
    save_file.open(filename.c_str());

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
    std::vector<Eigen::Vector3d> trajectory(2),pos_go,acc_go,pos_ret,acc_ret;
    trajectory[0] = Eigen::Vector3d(0,0,0);

    trajectory[1] = Eigen::Vector3d(0,displacement,0);


    //trajectory[2] = Eigen::Vector3d(0.4,0.2,0);
    //trajectory[3] = Eigen::Vector3d(0.2,0.1,0.1);
    //trajectory[2] = Eigen::Vector3d(-0.1,0.1,0);
    //trajectory[3] = Eigen::Vector3d(0,0,0);


    //object_pose

    




    for(int i=0;i<loop_number;i++){
        //direct motion
        ac.sendGoal(goal);
        ac.waitForResult();
        result = ac.getResult()->position;
        write_to_file(save_file,result);
        
        Eigen::Vector3d object_pose(result.data[0],result.data[1],result.data[2]);
        line_control.initialiseSystem(trajectory,object_pose,filter_parameters,fir_parameters,sampling,compensation);
        pos_go = line_control.getTrajPos();
        acc_go = line_control.getTrajAcc();
        line_control.run_continuous(compensation,pos_go,acc_go);

        //reverse motion
        ac.sendGoal(goal);
        ac.waitForResult();
        result = ac.getResult()->position;
        write_to_file(save_file,result);
        object_pose = Eigen::Vector3d(result.data[0],result.data[1],result.data[2]);
        trajectory[1] = Eigen::Vector3d(0,-displacement,0);
        line_control.initialiseSystem(trajectory,object_pose,filter_parameters,fir_parameters,sampling,compensation);
        pos_ret = line_control.getTrajPos();
        acc_ret = line_control.getTrajAcc();

        save_file<<std::endl;

        line_control.run_continuous(compensation,pos_ret,acc_ret);
        save_file<<std::endl;
    }
    save_file.close();


    return 0;
}
