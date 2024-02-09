/* 
 * File:   SS_filter.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef SS_LINE_CONTROL
#define SS_LINE_CONTROL

#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include "ss_exponential_filter/SS_filter.h"
#include "ss_exponential_filter/ComauIK.h"
#include "ss_exponential_filter/Liquid_handler.h"
#include "ss_exponential_filter/FIRFilter.h"

namespace ss_exponential_filter {

    class SS_line_control
    {
        public:

        SS_line_control(ros::NodeHandle &n,std::vector<std::string> &topics);			    //Class Constructor 
        virtual ~SS_line_control();			                                                                //Class Destructor


        //Methods
        int publishRobotReference();
        bool ikCheck();
        void run(bool angular_compensation);
        void run_continuous(bool angular_compensation,std::vector<Eigen::Vector3d> filtered_traj,std::vector<Eigen::Vector3d> filtered_acc);
        void initialiseRobotFilteredTrajectory();
        void initialiseRobotDirectTrajectory();
        void initialiseSystem(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d object_pose,std::vector<double> filter_parameters,std::vector<double> fir_period,double fir_freq,int filter_curve);
        void robot_callback(const sensor_msgs::JointState &state);
        void initialiseRos(std::vector<std::string> topics);
        void initialiseControlSubscriber(std::string &control_subscriber_topic);
        void initialiseRobotSubscriber(std::string &robot_subscriber_topic);
        void initialiseRobotPublisher(std::string &robot_publisher_topic);
        std::vector<Eigen::Vector3d> getTrajPos();
        std::vector<Eigen::Vector3d> getTrajAcc();
        //set functions

        
        //get functions


        private:

        //class variables
        ros::NodeHandle nh;                                 //Ros nodehandle from main
        ros::Subscriber robot_sub;                          //subscriber to robot joint states
        ros::Publisher robot_pub;                           //publisher for robot references
        ros::Publisher ef_pose_pub;                         //publisher for the robot ee poses
        ros::Publisher filter_pub;  
        std::vector<Eigen::Vector3d> control_reference;                  //the new reference point from the operator
        std::vector<Eigen::Vector3d> filter_reference;
        bool start;
        Eigen::Vector3d robot_ef_state;                     //position of the robot end effector in the workspace        
        std::vector<Eigen::Vector3d> filtered_traj;    //the filtered reference point for the robot
        std::vector<Eigen::Vector3d> filtered_acc;
        std::vector<double> trajectory_check;               //the reference control values to avoid jumps
        std::vector<double> trajectory_reference;           //the new reference joint state position for the robot
        std::vector<double> robot_joint_state;              //the new joint state position of the robot
        Eigen::Vector3d trajectory_offset;
        ss_exponential_filter::ComauIK *robot_ik;           //ik class for comau robot
        ss_exponential_filter::SS_filter *ss_filter;        //sloshing_suppression_filter
        ss_exponential_filter::Liquid_handler *tool;        //class for force sensor measurements
        ss_exponential_filter::FIRFilter *fir_filter;
        ss_exponential_filter::FIRFilter *fir_filter2;
        
        //class methods


  };
}

#endif /* SS_LINE_CONTROL */
