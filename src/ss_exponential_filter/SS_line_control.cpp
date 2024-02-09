/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SS_filter.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Jen 17, 2018, 4:05 PM
 */


#include <stdio.h>
#include <math.h>
#include <iostream>
#include "SS_line_control.h"


namespace ss_exponential_filter  {
	
    	SS_line_control::SS_line_control(ros::NodeHandle &n,std::vector<std::string> &topics):nh(n) {

            this->robot_ik = new ss_exponential_filter::ComauIK();
            this->ss_filter = new ss_exponential_filter::SS_filter();
            this->fir_filter = new ss_exponential_filter::FIRFilter();
            this->fir_filter2 = new ss_exponential_filter::FIRFilter();
            this->robot_ef_state = Eigen::Vector3d(0.6,0,1.4);
            this->trajectory_reference.resize(6);
            this->start = false;
            this->robot_joint_state.resize(6);
            this->control_reference.push_back(Eigen::Vector3d(0,0,0));
            std::fill(this->trajectory_reference.begin(),this->trajectory_reference.end(),0.0);
            std::fill(this->robot_joint_state.begin(),this->robot_joint_state.end(),0.0);
            initialiseRos(topics);
    	}


    	SS_line_control::~SS_line_control() {
    	}


      void SS_line_control::initialiseRobotPublisher(std::string &robot_publisher_topic){
          this->robot_pub=this->nh.advertise<sensor_msgs::JointState>(robot_publisher_topic.c_str(), 1, true);
          this->filter_pub =this->nh.advertise<sensor_msgs::JointState>("filter_acc",1,true);
          this->ef_pose_pub=this->nh.advertise<geometry_msgs::Pose>("comau_smart_six/robot_ef_pose",1,true);
      }

      void SS_line_control::initialiseRobotSubscriber(std::string &robot_subscriber_topic){
          this->robot_sub=this->nh.subscribe(robot_subscriber_topic.c_str(), 1,&SS_line_control::robot_callback,this);
      }

      void SS_line_control::initialiseRos(std::vector<std::string> topics){
          if (topics.size()>=2){
              initialiseRobotPublisher(topics[0]);
              initialiseRobotSubscriber(topics[1]);
          }
      }
      void SS_line_control::robot_callback(const sensor_msgs::JointState &state){

          for (int i=0;i<6;i++){
              this->robot_joint_state[i]=state.position[i];
          }
        this->robot_ef_state = this->robot_ik->fk_eigen(this->robot_joint_state);
        this->start = true;
      }
      void SS_line_control::initialiseSystem(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d object_pose,std::vector<double> filter_parameters,std::vector<double> fir_period,double fir_freq,int filter_curve){
          //set harmonic smoother parameters
          this->ss_filter->setFilterParameters(filter_parameters);
          this->ss_filter->setFilterCurve(filter_curve);
          //set fir filters parameters
          this->fir_filter->setFilterParameters(fir_period[0],fir_freq);
          this->fir_filter2->setFilterParameters(fir_period[1],fir_freq);
          //initialise fir filter cells
          this->fir_filter->initialiseFilter(trajectory[0]);
          this->fir_filter2->initialiseFilter(trajectory[0]);
          //set object offset for ik control 
          this->robot_ik->setToolOffsets(object_pose);
          this->control_reference.clear();
          this->control_reference = trajectory;
      }

      void SS_line_control::initialiseRobotFilteredTrajectory(){
          Eigen::Vector3d temp_state ;
          int cell_number = this->fir_filter->getFilterCellNumber();
          std::vector<double> extrapolated_parameters;
          std::vector<double> evaluated_parameters;
          this->robot_ik->reset();
          this->trajectory_check=this->robot_joint_state;
          for (int j=0;j<this->control_reference.size();j++){
              for (int i=0;i<cell_number;i++){
                  this->fir_filter->addNewInput(this->control_reference[j]);
                  this->fir_filter2->addNewInput(this->fir_filter->evaluateFilterOutput());
                  this->filter_reference.push_back(this->fir_filter2->evaluateFilterOutput());
              }
          }
          for (int i=0;i<cell_number;i++){
              this->fir_filter->addNewInput(this->control_reference[this->control_reference.size()-1]);
              this->fir_filter2->addNewInput(this->fir_filter->evaluateFilterOutput());
              this->filter_reference.push_back(this->fir_filter2->evaluateFilterOutput());
          }
          std::cout<<"fir_filter"<<this->filter_reference.size()<<std::endl;
          temp_state = this->robot_ik->fk_eigen(this->robot_joint_state);
          this->ss_filter->evaluateFilteredTrajectory(this->filter_reference,temp_state);
          //this->filtered_traj = this->filter_reference;
          this->filtered_traj = this->ss_filter->getFilteredTrajectory();
          this->filtered_acc = this->ss_filter->getFilteredAcceleration();
          std::cout<<"dimensione"<<this->filtered_traj.size()<<std::endl;
      }

      void SS_line_control::initialiseRobotDirectTrajectory(){
          Eigen::Vector3d temp_state ;
          int cell_number = this->ss_filter->getFilterSampleNumber();
          std::vector<Eigen::Vector3d> reference; 
          this->robot_ik->reset();
          this->trajectory_check=this->robot_joint_state;
          for (int j=0;j<this->control_reference.size();j++){
              for (int i=0;i<cell_number;i++){
                  reference.push_back(this->control_reference[j]);
              }
          }
          std::cout<<"fir_filter"<<this->filter_reference.size()<<std::endl;
          temp_state = this->robot_ik->fk_eigen(this->robot_joint_state);
          this->ss_filter->evaluateFilteredTrajectory(reference,temp_state);
          //this->filtered_traj = this->filter_reference;
          this->filtered_traj = this->ss_filter->getFilteredTrajectory();
          this->filtered_acc = this->ss_filter->getFilteredAcceleration();
          std::cout<<"dimensione"<<this->filtered_traj.size()<<std::endl;
      }

      int SS_line_control::publishRobotReference(){
          sensor_msgs::JointState state_msg;
          state_msg.position.resize(6);
          for (int i=0;i<6;i++){
              state_msg.position[i]=this->trajectory_reference[i];
          }
          //if (state_msg.position[4]<-1.8) state_msg.position[4]=-1.8;
          if(!ikCheck()){
            std::cout<<"big angles difference, stopping"<<std::endl;
            return -1;
          }
          ros::Time tmsg=ros::Time::now();
          state_msg.header.stamp.sec=tmsg.sec;
          state_msg.header.stamp.nsec=tmsg.nsec;
          this->robot_pub.publish(state_msg);
      }

      bool SS_line_control::ikCheck(){
          if (this->trajectory_check.empty()){
              for (int i=0;i<6;i++){
                  this->trajectory_check.push_back(this->trajectory_reference[i]);
              }
              return true;
          }
          else{
              for(int i=0;i<6;i++){
                  if(fabs(this->trajectory_check[i]-this->trajectory_reference[i])>0.2){
                      std::cout<<"i "<<i<<std::endl;
                      std::cout<<"check "<<this->trajectory_check[i]<<std::endl;
                      std::cout<<"ref "<<this->trajectory_reference[i]<<std::endl;
                      return false;
                  }
                  this->trajectory_check[i]=this->trajectory_reference[i];
              }
              return true;
          }
      }

      void SS_line_control::run(bool angular_compensation){
        KDL::Frame target_frame;
        sensor_msgs::JointState state_msg;
        geometry_msgs::Pose robot_ef_pose;
        double quat_x,quat_y,quat_z,quat_w;
        state_msg.position.resize(6);
        bool first = true;
        int counter = 0;
        int end = this->filtered_traj.size();
        ros::Rate r(500);
        while(ros::ok()){
            if (start){
              if(first){
                this->initialiseRobotFilteredTrajectory();
                first = false;
              }
              for (int i=0;i<3;i++){
                state_msg.position[i] = filtered_traj[counter][i];
                state_msg.position[i+3] =filtered_acc[counter][i];
              }
              ros::Time tmsg=ros::Time::now();
              state_msg.header.stamp.sec=tmsg.sec;
              state_msg.header.stamp.nsec=tmsg.nsec;
              this->filter_pub.publish(state_msg);
            
              target_frame.p=KDL::Vector(1000*this->filtered_traj[counter][0],1000*this->filtered_traj[counter][1],1000*this->filtered_traj[counter][2]);
              //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*this->filtered_traj[counter][1],1000*this->filtered_traj[counter][0]));
              target_frame.M=KDL::Rotation::RPY(0,0,0);

              this->robot_ik->ik_neg5(target_frame,this->filtered_acc[counter],angular_compensation,this->trajectory_reference);
              counter++;

              //publish end effector pose
              robot_ef_pose.position.x = target_frame.p[0];
              robot_ef_pose.position.y = target_frame.p[1];
              robot_ef_pose.position.z = target_frame.p[2];
              target_frame.M.GetQuaternion(quat_x,quat_y,quat_z,quat_w);
              robot_ef_pose.orientation.x = quat_x;
              robot_ef_pose.orientation.y = quat_y;
              robot_ef_pose.orientation.z = quat_z;
              robot_ef_pose.orientation.z = quat_w;
              this->ef_pose_pub.publish(robot_ef_pose);


              //publish robot reference
              this->publishRobotReference();
              if (counter>= this->filtered_traj.size()){
                ros::shutdown();
              }
            }
            ros::spinOnce();
            r.sleep();
        }
      }

      void SS_line_control::run_continuous(bool angular_compensation,std::vector<Eigen::Vector3d> filtered_traj,std::vector<Eigen::Vector3d> filtered_acc){
        KDL::Frame target_frame;
        sensor_msgs::JointState state_msg;
        state_msg.position.resize(6);
        bool first = true;
        int counter = 0;
        int end = this->filtered_traj.size();
        ros::Rate r(500);
        while(ros::ok()){
            if (start){
              if(first){
                this->initialiseRobotFilteredTrajectory();
                first = false;
              }
              for (int i=0;i<3;i++){
                state_msg.position[i] = filtered_traj[counter][i];
                state_msg.position[i+3] =filtered_acc[counter][i];
              }
              ros::Time tmsg=ros::Time::now();
              state_msg.header.stamp.sec=tmsg.sec;
              state_msg.header.stamp.nsec=tmsg.nsec;
              this->filter_pub.publish(state_msg);
            
              target_frame.p=KDL::Vector(1000*filtered_traj[counter][0],1000*filtered_traj[counter][1],1000*filtered_traj[counter][2]);
              //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*this->filtered_traj[counter][1],1000*this->filtered_traj[counter][0]));
              target_frame.M=KDL::Rotation::RPY(0,0,0);
              this->robot_ik->ik_neg5(target_frame,filtered_acc[counter],angular_compensation,this->trajectory_reference);
              counter++;

              this->publishRobotReference();
              if (counter>= this->filtered_traj.size()){
                ros::shutdown();
              }
            }
            ros::spinOnce();
            r.sleep();
        }
      }

      std::vector<Eigen::Vector3d> SS_line_control::getTrajPos(){
        return this->filtered_traj;
      }

      std::vector<Eigen::Vector3d> SS_line_control::getTrajAcc(){
        return this->filtered_acc;
      }
}


