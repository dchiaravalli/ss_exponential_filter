/* 
 * File:   SS_filter.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef SS_OFFLINE_CONTROL
#define SS_OFFLINE_CONTROL

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



namespace ss_exponential_filter {

    class SS_offline_control
    {
        public:

        SS_offline_control(ros::NodeHandle &n,std::vector<std::string> &topics,std::string mode);			    //Class Constructor 
        virtual ~SS_offline_control();			                                                                //Class Destructor

        //state list
        enum State{
            READING,
            START,
            RUNNING,
            CHECKING,
            WAITING,
            STALLING,
            INITIALIZING,
            FILTERING,
            SETTING,
            STOP
        };

        //Methods
        void userKeyboardEventControl();
        void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out);
        void evaluateNewTrajectoryReference(bool angular_compensation,bool stabilization);
        void evaluateUnfilteredReference();
        int publishRobotReference();
        int spin();
        std::thread inputThread();

        //set functions
        void setSaveFilePath(std::string filepath);
        void setSourceTrajectoryFilePath(std::string filepath);
        void setSSFilterParameters(std::vector<double> parameters);
        void setRobotIkOffsets(Eigen::Vector3d offsets);
        
        //get functions
        std::string getSaveFilePath();
        
        private:

        //class variables
        ros::NodeHandle nh;                                 //Ros nodehandle from main
        ros::Subscriber robot_sub;                          //subscriber to robot joint states
        ros::Subscriber control_sub;                        //subscriber to operator movements
        ros::Publisher robot_pub;                           //publisher for robot references
        std::string mode;                                   //operator tracking mode
        std::stringstream save_file_path;                   //path location for saving trajectories
        std::stringstream source_file_path;                 //path location for source trajectories
        std::ofstream save_file;                            //save file for trajectories
        tf::TransformListener listener;                     //listener to vicon system
        tf::StampedTransform  transform;                    //support transform for vicon reception
        Eigen::Vector3d control_reference;                  //the new reference point from the operator
        std::vector<Eigen::Vector3d> source_control_reference;
        std::vector<Eigen::Vector3d> inverse_control_reference;
        Eigen::Vector3d reference_stabilizer;
        Eigen::Vector3d robot_ef_state;                     //position of the robot end effector in the workspace        
        std::vector<Eigen::Vector3d> filtered_reference;    //the filtered reference point for the robot
        std::vector<double> trajectory_check;               //the reference control values to avoid jumps
        std::vector<double> trajectory_reference;           //the new reference joint state position for the robot
        std::vector<double> robot_joint_state;              //the new joint state position of the robot
        Eigen::Vector3d trajectory_offset;
        bool isGoing;                                       //check if new trajectory has started
        bool advanced_control;
        bool go_with_traj;
        bool homing_complete;
        bool doppio;
        bool reading;
        int index;
        State state;
        ss_exponential_filter::ComauIK *robot_ik;           //ik class for comau robot
        ss_exponential_filter::SS_filter *ss_filter;        //sloshing_suppression_filter
        ss_exponential_filter::Liquid_handler *tool;        //class for force sensor measurements
        int loop_counter;                                   //keep count of the final stabilizing cycles
        typedef void (SS_offline_control::*state_pointer)();
        state_pointer state_machine;
        
        //class methods
        void initialiseRobotPublisher(std::string &robot_publisher_topic);
        void initialiseRobotSubscriber(std::string &robot_subscriber_topic);
        void initialiseControlSubscriber(std::string &control_subscriber_topic);
        void initialiseRos(std::vector<std::string> topics);
        void robot_callback(const sensor_msgs::JointState &state);
        void control_callback(const sensor_msgs::JointState &state);
        void initialiseRobotFilteredTrajectory();
        void initialiseRobotTrajectory();
        bool ikCheck();
        void evaluateLiquidParameters();
        void lol();
  };
}

#endif /* SS_OFFLINE_CONTROL */
