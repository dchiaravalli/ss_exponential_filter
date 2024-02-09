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
#include "SS_filter.h"

namespace ss_exponential_filter  {
	

	
    	SS_filter::SS_filter(double period,double sampling_time, double deformedImpulseResponseFactor) {
			this->period=period;						
			this->sampling_time=sampling_time;	
			this->filter_response_shaper=deformedImpulseResponseFactor;			
            this->sample_number=round(this->period/this->sampling_time);
			this->filter_initial_state=Eigen::Vector3d(0,0,0);
			this->filter_position_offset=Eigen::Vector3d(0,0,0);
      		this->filter_curve = 0;
			this->safety_offset=Eigen::Vector3d(0,0,0);
            inputVectorInitialiser();
            outputVectorInitialiser();
			evaluateFilterConstants();
    	}

    	SS_filter::~SS_filter() {
    	}

		void SS_filter::inputVectorInitialiser(){
			input_vector.clear();
			for (int i=0;i<this->sample_number+3;i++)
				input_vector.push_back(this->filter_initial_state);
		}

		void SS_filter::outputVectorInitialiser(){
			output_pos_vector.clear();
			output_acc_vector.clear();
			for (int i=0; i<2; i++)
			{
				output_pos_vector.push_back(this->filter_initial_state);
				output_acc_vector.push_back(Eigen::Vector3d(0,0,0));
			}
		}

		void SS_filter::evaluateFilterConstants(){
			double a,b,c,d;
			a = exp(this->filter_response_shaper*this->sampling_time);
			b = cos(M_PI/this->sample_number);
			c = sin(M_PI/this->sample_number);
			d = this->filter_response_shaper/(M_PI/this->period);
			this->I = a*(b-d*c);
			this->A = 1-I;
			this->F = exp(2*this->filter_response_shaper*this->sampling_time);
			this->C = exp(this->filter_response_shaper*this->sampling_time*this->sample_number);
			this->B = F + a * (-b-d*c);
			this->D = 1/(1+C);
			this->E = 2*a*b;
			this->H = (pow((M_PI/this->period),2)+pow(this->filter_response_shaper,2));
		}

		void SS_filter::evaluateSingleFilterStep(Eigen::Vector3d &input){
			this->input_vector.erase(this->input_vector.begin());
			this->input_vector.push_back(input);
			this->output_pos_vector.push_back(evaluateFilterPositionFunction());
			this->output_acc_vector.push_back(evaluateFilterAccelerationFunction());
			this->output_pos_vector.erase(this->output_pos_vector.begin());
			this->output_acc_vector.erase(this->output_acc_vector.begin());
			this->filtered_trajectory.push_back(this->output_pos_vector[1]);
			this->filtered_acceleration.push_back(this->output_acc_vector[1]);
		}

		std::vector<Eigen::Vector3d> SS_filter::evaluateOnlineSingleFilterStep(Eigen::Vector3d &input){
			std::vector<Eigen::Vector3d> filter_step;
			//std::cout<<input<<std::endl;
			Eigen::Vector3d safety=input-this->filter_position_offset;
			/*if (safety[0]<0.6+this->safety_offset[0]) safety[0]=0.6+this->safety_offset[0];
			if (safety[0]>1.2+this->safety_offset[0]) safety[0]=1.2+this->safety_offset[0];
			if (safety[1]<-0.4+this->safety_offset[1]) safety[1]=-0.4+this->safety_offset[1];
			if (safety[1]>0.4+this->safety_offset[1]) safety[1]=0.4+this->safety_offset[1];
			if (safety[2]<0.6+this->safety_offset[2]) safety[2]=0.6+this->safety_offset[2];
			if (safety[2]>1.2+this->safety_offset[2]) safety[2]=1.2+this->safety_offset[2];*/
			/*if (safety[0]<0.6) safety[0]=0.6;
			if (safety[0]>1.2) safety[0]=1.2;
			if (safety[1]<-0.4) safety[1]=-0.4;
			if (safety[1]>0.4) safety[1]=0.4;
			if (safety[2]<0.5) safety[2]=0.5;
			if (safety[2]>1.4) safety[2]=1.4;*/
			//std::cout<<safety<<std::endl;
			this->input_vector.erase(this->input_vector.begin());
			this->input_vector.push_back(safety);
			filter_step.push_back(evaluateFilterPositionFunction());
			filter_step.push_back(evaluateFilterAccelerationFunction());
			this->output_pos_vector.erase(this->output_pos_vector.begin());
			this->output_pos_vector.push_back(filter_step[0]);
			this->output_acc_vector.erase(this->output_acc_vector.begin());
			this->output_acc_vector.push_back(filter_step[1]);
			return filter_step;
		}

		void SS_filter::evaluateFilteredTrajectory(std::vector<Eigen::Vector3d> &trajectory){
			setFilterInitialState(trajectory[0]);
			int iterations = trajectory.size();
			for (int i=0;i<trajectory.size();i++){
				evaluateSingleFilterStep(trajectory[i]);
			}
			for(int i=0;i<this->sample_number;i++){
				evaluateSingleFilterStep(trajectory[iterations-1]);
			}
		}

		void SS_filter::evaluateFilteredTrajectory(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d &initial_state){
			Eigen::Vector3d temp_point;
			setFilterInitialState(initial_state);
			this->filter_position_offset=trajectory[0]-this->filter_initial_state;
			for (int i=0;i<trajectory.size();i++){
				temp_point=trajectory[i]-this->filter_position_offset;
				evaluateSingleFilterStep(temp_point);
			}
			for(int i=0;i<this->sample_number;i++){
				evaluateSingleFilterStep(temp_point);
			}
		}

		void SS_filter::evaluateViaPointTrajectory(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d &initial_state){
			Eigen::Vector3d temp_point;
			setFilterInitialState(initial_state);
			this->filter_position_offset=trajectory[0]-this->filter_initial_state;
			for (int i=0;i<trajectory.size();i++){
				for (int j=0;j<this->sample_number-this->filter_curve;j++){
					temp_point=trajectory[i]-this->filter_position_offset;
					evaluateSingleFilterStep(temp_point);
				}
			}
			//final convergence
			for(int i=0;i<this->sample_number;i++){
				evaluateSingleFilterStep(temp_point);
			}
		}

		Eigen::Vector3d SS_filter::evaluateFilterPositionFunction(){
			Eigen::Vector3d first = this->E*this->output_pos_vector[1]-this->F*this->output_pos_vector[0];
			Eigen::Vector3d second = this->D*this->A*(this->input_vector[this->sample_number+1]+this->C*this->input_vector[1]);
			Eigen::Vector3d third = this->D*this->B*(this->input_vector[this->sample_number]+this->C*this->input_vector[0]);
			return first + second + third;
		}

		Eigen::Vector3d SS_filter::evaluateFilterAccelerationFunction(){
			Eigen::Vector3d first = this->E*this->output_acc_vector[1]-this->F*this->output_acc_vector[0];
			Eigen::Vector3d second = this->input_vector[this->sample_number+2]+this->C*this->input_vector[2];
			Eigen::Vector3d third = -(this->I+1)*(this->input_vector[this->sample_number+1]+this->C*this->input_vector[1]);
			Eigen::Vector3d fourth = this->I*(this->input_vector[this->sample_number]+this->C*this->input_vector[0]);
			return first + this->D*this->H*(second + third + fourth);
		}

		std::vector<Eigen::Vector3d> SS_filter::jointTrajectoryToEigen(trajectory_msgs::JointTrajectory &trajectory){
			std::vector<Eigen::Vector3d> result;
			for (int i=0;i<trajectory.points.size();i++){
				result.push_back(Eigen::Vector3d(trajectory.points[i].positions[0],trajectory.points[i].positions[1],trajectory.points[i].positions[2]));
			}
			return result;
		}

		Eigen::Vector3d SS_filter::jointTrajectoryPointToEigen(trajectory_msgs::JointTrajectoryPoint &point){
			return Eigen::Vector3d(point.positions[0],point.positions[1],point.positions[2]);
		}

		trajectory_msgs::JointTrajectory SS_filter::EigenToJointTrajectoryPos(std::vector<Eigen::Vector3d> positions){
			trajectory_msgs::JointTrajectory trajectory;
			trajectory.points.resize(positions.size());
			for (int i=0;i<positions.size();i++){
				trajectory.points[i].positions.resize(3);
				for (int j=0;j<3;j++)
					trajectory.points[i].positions[j]=positions[i][j];
			}
			return trajectory;
		}

		trajectory_msgs::JointTrajectory SS_filter::EigenToJointTrajectoryAcc(std::vector<Eigen::Vector3d> accelerations){
			trajectory_msgs::JointTrajectory trajectory;
			trajectory.points.resize(accelerations.size());
			for (int i=0;i<accelerations.size();i++){
				trajectory.points[i].accelerations.resize(3);
				for (int j=0;j<3;j++)
					trajectory.points[i].accelerations[j]=accelerations[i][j];
			}
			return trajectory;
		}

		trajectory_msgs::JointTrajectory SS_filter::EigenToJointTrajectory(std::vector<Eigen::Vector3d> positions,std::vector<Eigen::Vector3d> accelerations){
			trajectory_msgs::JointTrajectory trajectory;
			assert(positions.size()==accelerations.size());
			trajectory.points.resize(positions.size());
			for (int i=0;i<accelerations.size();i++){
				trajectory.points[i].positions.resize(3);
				trajectory.points[i].accelerations.resize(3);
				for (int j=0;j<3;j++){
					trajectory.points[i].positions[j]=positions[i][j];
					trajectory.points[i].accelerations[j]=accelerations[i][j];
				}
			}
			return trajectory;
		}

		trajectory_msgs::JointTrajectoryPoint SS_filter::EigenToJointTrajectorPoint(Eigen::Vector3d position,Eigen::Vector3d acceleration){
			trajectory_msgs::JointTrajectoryPoint point;
			point.positions.resize(3);
			point.accelerations.resize(3);
			for (int j=0;j<3;j++){
				point.positions[j]=position[j];
				point.accelerations[j]=acceleration[j];
			}
			return point;
		}

		void SS_filter::setFilterPeriod(double period){
			this->period=period;
			this->sample_number=round(this->period/this->sampling_time);
			inputVectorInitialiser();
			evaluateFilterConstants();
		}

		void SS_filter::setFilterSamplingTime(double sampling_time){
			this->sampling_time=sampling_time;
			this->sample_number=round(this->period/this->sampling_time);
			inputVectorInitialiser();	
			evaluateFilterConstants();		
		}

		void SS_filter::setFilterResponseShaper(double filter_response_shaper){
			this->filter_response_shaper=filter_response_shaper;
		}

    void SS_filter::setFilterCurve(int filter_curve){
      this->filter_curve = filter_curve;
    }
		void SS_filter::setFilterInitialState(Eigen::Vector3d &initial_state){
			this->filter_initial_state=initial_state;
			resetFilterState();
		}

		void SS_filter::setFilterPositionOffset(Eigen::Vector3d &position_offset){
			this->filter_position_offset=position_offset;
		}

		void SS_filter::setFilterParameters(double period, double sampling_time, double filter_response_shaper){
			this->period=period;
			this->sampling_time=sampling_time;
			this->filter_response_shaper=filter_response_shaper;
			this->sample_number=round(this->period/this->sampling_time);
			inputVectorInitialiser();
			evaluateFilterConstants();
		}

		void SS_filter::setFilterParameters(std::vector<double> parameters){
			this->period=parameters[0];
			this->sampling_time=parameters[1];
			this->filter_response_shaper=parameters[2];
			this->sample_number=round(this->period/this->sampling_time);
			inputVectorInitialiser();
			evaluateFilterConstants();
		}

		void SS_filter::resetFilterState(){
			inputVectorInitialiser();
			outputVectorInitialiser();
			this->filter_position_offset=Eigen::Vector3d(0,0,0);
			this->filtered_trajectory.clear();
			this->filtered_acceleration.clear();
		}

		void SS_filter::setSafetyOffset(Eigen::Vector3d safety_offset){
				this->safety_offset=safety_offset;//+0.01*Eigen::Vector3d(1,1,1);
		}

		const double SS_filter::getFilterPeriod(){
			return this->period;
		}

		const double SS_filter::getFilterSamplingTime(){
			return this->sampling_time;
		}

		const double SS_filter::getFilterResponseShaper(){
			return this->filter_response_shaper;
		}

		const double SS_filter::getFilterSampleNumber(){
			return this->sample_number;
		}

		const Eigen::Vector3d SS_filter::getFilterInitialState(){
			return this->filter_initial_state;
		}

		std::vector<double> SS_filter::getFilterParameters(){
			std::vector<double> parameters(3);
			parameters[0]=this->period;
			parameters[1]=this->sampling_time;
			parameters[2]=this->filter_response_shaper;
			return parameters;
		}

		std::vector<Eigen::Vector3d> SS_filter::getFilteredTrajectory(){
			return this->filtered_trajectory;
		}

		std::vector<Eigen::Vector3d> SS_filter::getFilteredAcceleration(){
			return this->filtered_acceleration;
		}
}
