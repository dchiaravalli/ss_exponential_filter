/* 
 * File:   Liquid_handler.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef LIQUID_HANDLER_H
#define LIQUID_HANDLER_H

#include <ros/ros.h>
#include <fstream>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include "ss_exponential_filter/Tool_posefinder.h"
#include "ss_exponential_filter/SS_parameter_evaluator.h"
#include "modelsignal/modelNoiseSignal.h"
#include "ss_exponential_filter/force_object_pose.h"


namespace ss_exponential_filter {


    class Liquid_handler
    {
        public:

        Liquid_handler(ros::NodeHandle &n,std::string force_topic,int sampling_number=1000,bool reverse=false);			    //Class Constructor 
        virtual ~Liquid_handler();			                                                                //Class Destructor

        //state list
        enum Sensor_state{
            INITIALIZE,
            READY,
            STABILIZING,
            MEASURING,
            PRESENT,
        };
        //Methods


        //set functions
        void setSamplingNumber(int sampling_number);
        void setEnableFinder();
        //get functions
        Sensor_state getSensorState();
        Eigen::Vector3d getMinForceMeasure();
        Eigen::Vector3d getMaxForceMeasure();
        Eigen::Vector3d getMinTorqueMeasure();
        Eigen::Vector3d getMaxTorqueMeasure();
        std::vector<Eigen::Vector3d> getMinMeasure();
        std::vector<Eigen::Vector3d> getMaxMeasure();
        std::vector<double> getFilterParameters();
        std::vector<double> extrapolateLiquidParameters();
        Eigen::Vector3d getToolOffset();
        Eigen::Vector3d getToolOffsetReverse();
        Eigen::Vector3d getToolRigidObjectReverse();
        Eigen::Vector3d getFixedPose();
        Sensor_state state;
        private:

        //class variables
        ros::NodeHandle nh;                                 //Ros nodehandle from main
        ros::Subscriber force_sub;                          //subscriber to force sensor
        ros::ServiceClient model_signal_client;
        ros::ServiceServer get_pose_service;
        ss_exponential_filter::Tool_posefinder *posefinder;
        ss_exponential_filter::SS_parameter_evaluator *evaluator;
        modelsignal::modelNoiseSignal srv;
        Eigen::Vector3d mean_force_min;
        Eigen::Vector3d mean_force_max;
        Eigen::Vector3d mean_torque_min;
        Eigen::Vector3d mean_torque_max;
        std::vector<double> periodic_signal;
        int sampling_number;
        int mean_counter;
        bool enable_finder;
        int liquid_period;
        int stabilizer_wait;
        
        bool isReady;
        bool hasLiquid;
        bool reverse;

        //class methods
        void initialiseForceSubscriber(std::string force_subscriber_topic);
        void initialiseModelSignalClient(std::string model_signal_topic);
        void force_callback(const geometry_msgs::Twist &force);
        void force_callback_reverse(const geometry_msgs::Twist &force);
        bool service_callback(ss_exponential_filter::force_object_pose::Request &req,ss_exponential_filter::force_object_pose::Response &res);
        void resetMeanMeasurements();
        void resetMeanReferences();
        void setNewSensorInput(geometry_msgs::Twist sensor);
        void setNewSensorInputReversed(geometry_msgs::Twist sensor);
        void evaluateLiquidParameters();
    };
}

#endif /* LIQUID_HANDLER_H */
