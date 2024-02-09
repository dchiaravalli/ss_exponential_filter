/* 
 * File:   SS_filter.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef SS_ONLINE_CONTROL
#define SS_ONLINE_CONTROL

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

    class SS_online_control
    {
        public:

        SS_online_control(ros::NodeHandle &n,std::vector<std::string> &topics,std::string mode);			    //Class Constructor 
        virtual ~SS_online_control();			                                                                //Class Destructor

        //state list
        enum State{
            READING,
            START,
            RUNNING,
            CHECKING,
            INITIALIZING,
            FILTERING,
            SETTING,
            STOP
        };

        //Methods
        void checkVicon(std::string base_ref,std::string new_ref);
        void updateViconReference(std::string &base_ref,std::string &new_ref,float threshold = 0.3);
        void userKeyboardEventControl();
        void writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out);
        void evaluateNewTrajectoryReference(bool angular_compensation,bool stabilization);
        void evaluateUnfilteredReference();
        int publishRobotReference();
        int spin(std::string base_ref,std::string new_ref,float threshold = 0.3);
        std::thread inputThread();

        //set functions
        void setSaveFilePath(std::string filepath);
        void setSSFilterParameters(std::vector<double> parameters);
        void setRobotIkOffsets(Eigen::Vector3d offsets);
        void setViconSaturation();
        void setWorkspaceSaturation(Eigen::Vector3d min_saturation,Eigen::Vector3d max_saturation);
        
        //get functions
        std::string getSaveFilePath();
        Eigen::Vector3d getMaxWorkspaceSaturation();
        Eigen::Vector3d getMinWorkspaceSaturation();

        private:

        //class variables
        ros::NodeHandle nh;                                 //Ros nodehandle from main
        ros::Subscriber robot_sub;                          //subscriber to robot joint states
        ros::Subscriber control_sub;                        //subscriber to operator movements
        ros::Publisher robot_pub;                           //publisher for robot references
        ros::Publisher filter_pub;
        std::string mode;                                   //operator tracking mode
        std::stringstream save_file_path;                   //path location for saving trajectories
        std::ofstream save_file;                            //save file for trajectories
        tf::TransformListener listener;                     //listener to vicon system
        tf::StampedTransform  transform;                    //support transform for vicon reception
        Eigen::Vector3d control_reference;                  //the new reference point from the operator
        Eigen::Vector3d reference_stabilizer;
        Eigen::Vector3d robot_ef_state;                     //position of the robot end effector in the workspace        
        std::vector<Eigen::Vector3d> filtered_reference;    //the filtered reference point for the robot
        std::vector<double> trajectory_check;               //the reference control values to avoid jumps
        std::vector<double> trajectory_reference;           //the new reference joint state position for the robot
        std::vector<double> robot_joint_state;              //the new joint state position of the robot
        Eigen::Vector3d trajectory_offset;
        Eigen::Vector3d min_workspace_saturation;
        Eigen::Vector3d max_workspace_saturation;   
        Eigen::Vector3d min_vicon_saturation;
        Eigen::Vector3d max_vicon_saturation;
        bool isGoing;                                       //check if new trajectory has started
        bool advanced_control;
        bool search_liquid;
        State state;
        ss_exponential_filter::ComauIK *robot_ik;           //ik class for comau robot
        ss_exponential_filter::SS_filter *ss_filter;        //sloshing_suppression_filter
        ss_exponential_filter::Liquid_handler *tool;        //class for force sensor measurements
        int loop_counter;                                   //keep count of the final stabilizing cycles
        typedef void (SS_online_control::*state_pointer)();
        state_pointer state_machine;
        
        //class methods
        void initialiseRobotPublisher(std::string &robot_publisher_topic);
        void initialiseRobotSubscriber(std::string &robot_subscriber_topic);
        void initialiseControlSubscriber(std::string &control_subscriber_topic);
        void initialiseRos(std::vector<std::string> topics);
        void robot_callback(const sensor_msgs::JointState &state);
        void control_callback(const geometry_msgs::Pose &msg);
        bool viconErrorControl(float threshold);
        bool controlErrorControl(geometry_msgs::Pose pose,float threshold = 0.3);
        void initialiseRobotFilteredTrajectory();
        void initialiseRobotTrajectory();
        bool ikCheck();
        void evaluateLiquidParameters();
        void viconSaturation();
        void lol();
  };
}

#endif /* SS_ONLINE_CONTROL */
