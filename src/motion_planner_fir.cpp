#include <ros/ros.h>
#include <Eigen/Dense>
#include <ss_exponential_filter/FIRFilter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "ss_exponential_filter/ComauIK.h"
#include <kdl/frames_io.hpp>

bool go = false;

std::vector<double> robot_joint_state;
std::vector<double> real_pose;
Eigen::Vector3d robot_ef_state;
ss_exponential_filter::ComauIK *robot_ik;
std::vector<double> trajectory_check;

void cb(const std_msgs::Float64MultiArray &msg)
{
    for(int i=0;i<3;i++){
        real_pose[i] = msg.data[i];
    }
    go =true;
}

void robot_callback(const sensor_msgs::JointState &state)
{
    for (int i=0;i<6;i++){
        robot_joint_state[i]=state.position[i];
    }
    robot_ef_state = robot_ik->fk_eigen(robot_joint_state);
    go = true;
}

bool ikCheck(std::vector<double> trajectory_reference){
    if (trajectory_check.empty()){
        for (int i=0;i<6;i++){
            trajectory_check.push_back(trajectory_reference[i]);
        }
        return true;
    }
    else{
        for(int i=0;i<6;i++){
            if(fabs(trajectory_check[i]-trajectory_reference[i])>0.2){
                std::cout<<"i "<<i<<std::endl;
                std::cout<<"check "<<trajectory_check[i]<<std::endl;
                std::cout<<"ref "<<trajectory_reference[i]<<std::endl;
                return false;
            }
            trajectory_check[i]=trajectory_reference[i];
        }
        return true;
    }
}

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "motion_planner_fir");
    ros::NodeHandle n;

    float obj_pose_y, acceleration;

    n.getParam("pos_y",obj_pose_y);
    n.getParam("acceleration",acceleration);

    float period = 2*fabs(obj_pose_y)/acceleration;

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("comau_smart_six/joint_command", 1, true);
    ss_exponential_filter::FIRFilter fir_filter_acc(period,0.001),fir_filter_vel(period,0.001),fir_filter_pos(2,0.001),filter4(2,0.001), filter5(2,0.001), filter6(2,0.001),filter7(2,0.001);
    ros::Subscriber sub = n.subscribe("cartesian_pose_controller/actual_cartesian_position",1,cb);
    ros::Subscriber robot_sub=n.subscribe("comau_smart_six/joint_states", 1,robot_callback);

    

    robot_ik=new ss_exponential_filter::ComauIK();
    Eigen::Vector3d final_pose(0.5,0.6,0.5),result;
    real_pose.resize(3);
    robot_joint_state.resize(6);
    std::fill(robot_joint_state.begin(),robot_joint_state.end(),0.0);
    sensor_msgs::JointState reference_joints;
    reference_joints.position.resize(6);
    std::vector<double> joint_sol;
    joint_sol.resize(6);
    Eigen::Vector3d robot_motion_start;
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    // while (n.ok()){
        
    //     try{
    //         transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_EE",
    //                                 ros::Time(0));
    //     }
    //     catch (tf2::TransformException &ex) {
    //         ROS_WARN("%s",ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }
    //     break;
    // }
    ros::Rate r(500);
    while(true)
    {
        if (go){
            //std::cout<<transformStamped.transform.translation.x<<std::endl;

            fir_filter_acc.initialiseFilter(Eigen::Vector3d(0,0,0));
            fir_filter_vel.initialiseFilter(Eigen::Vector3d(0,0,0));
            // fir_filter_pos.initialiseFilter(Eigen::Vector3d(0,0,0));
            // filter4.initialiseFilter(Eigen::Vector3d(0,0,0));
            // filter5.initialiseFilter(Eigen::Vector3d(0,0,0));
            // filter6.initialiseFilter(Eigen::Vector3d(0,0,0));
            // filter7.initialiseFilter(Eigen::Vector3d(0,0,0));
            break;
        }
        ros::spinOnce();
        r.sleep();
    }

    std::cout<<"ready to start"<<std::endl;
    std_msgs::Float64MultiArray delta_vector;
    KDL::Frame ik_frame;



    robot_motion_start = robot_ef_state;
    std::cout<<"pos "<<robot_motion_start<<std::endl;
    std::cout<<period<<std::endl;
    for (int i=0;i<period*2*1000;i++){
        std::cout<<"loop"<<i<<std::endl;
        fir_filter_acc.addNewInput(Eigen::Vector3d(0,obj_pose_y,0));
        fir_filter_vel.addNewInput(fir_filter_acc.evaluateFilterOutput());
        // fir_filter_pos.addNewInput(fir_filter_vel.evaluateFilterOutput());
        // filter4.addNewInput(fir_filter_pos.evaluateFilterOutput());
        // filter5.addNewInput(filter4.evaluateFilterOutput());
        // filter6.addNewInput(filter5.evaluateFilterOutput());
        // filter7.addNewInput(filter6.evaluateFilterOutput());
        result = fir_filter_vel.evaluateFilterOutput();
        ik_frame.p = KDL::Vector(1000*(robot_motion_start[0]),1000*(robot_motion_start[1]+result[1]),1000*(robot_motion_start[2]));
        ik_frame.M = KDL::Rotation::RPY(0,0,0);
        Eigen::Vector3d support(0,0,0);
        robot_ik->ik_neg5(ik_frame,support,false,joint_sol);
        if(!ikCheck(joint_sol)){
            std::cout<<"big angles difference, stopping"<<std::endl;
            return -1;
        }
        for(int j=0;j<6;j++)
        {
            reference_joints.position[j] = joint_sol[j];
        }
        ros::Time tmsg=ros::Time::now();
        reference_joints.header.stamp.sec=tmsg.sec;
        reference_joints.header.stamp.nsec=tmsg.nsec;
        pub.publish(reference_joints);
        ros::spinOnce();
        r.sleep();
        // reference.data[0] = transformStamped.transform.translation.x;
        // reference.data[1] = transformStamped.transform.translation.y;
        // reference.data[2] = transformStamped.transform.translation.z;
    }






    return 0;
}