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
#include "SS_offline_control.h"




namespace ss_exponential_filter  {
	
    	SS_offline_control::SS_offline_control(ros::NodeHandle &n,std::vector<std::string> &topics,std::string mode):nh(n) {
            this->mode=mode;
            this->isGoing=true;
            this->reading = false;
            this->advanced_control = false;
            this->go_with_traj=false;
            this->homing_complete = false;
            this->doppio=false;
            this->index=0;
            this->state = STOP;
            this->robot_ik = new ss_exponential_filter::ComauIK();
            this->ss_filter = new ss_exponential_filter::SS_filter();
            if (topics.size()==4){
                this->tool = new ss_exponential_filter::Liquid_handler(n,topics[3],2500,true);
                this->tool->setEnableFinder();

            }
            this->robot_ef_state = Eigen::Vector3d(0.6,0,1.4);
            this->loop_counter = this->ss_filter->getFilterSampleNumber();
            this->trajectory_reference.resize(6);
            this->robot_joint_state.resize(6);
            this->control_reference = Eigen::Vector3d(0,0,0);
            std::fill(this->trajectory_reference.begin(),this->trajectory_reference.end(),0.0);
            std::fill(this->robot_joint_state.begin(),this->robot_joint_state.end(),0.0);
            this->state_machine=&SS_offline_control::lol;
            initialiseRos(topics);
    	}

        void SS_offline_control::lol(){
            std::cout<<"lol"<<std::endl;
        }

    	SS_offline_control::~SS_offline_control() {
    	}

        void SS_offline_control::initialiseRobotPublisher(std::string &robot_publisher_topic){
            this->robot_pub=this->nh.advertise<sensor_msgs::JointState>(robot_publisher_topic.c_str(), 1, true);
        }

        void SS_offline_control::initialiseRobotSubscriber(std::string &robot_subscriber_topic){
            this->robot_sub=this->nh.subscribe(robot_subscriber_topic.c_str(), 1,&SS_offline_control::robot_callback,this);
        }

        void SS_offline_control::initialiseControlSubscriber(std::string &control_subscriber_topic){
            this->control_sub=this->nh.subscribe(control_subscriber_topic.c_str(), 1,&SS_offline_control::control_callback,this);
        }

        void SS_offline_control::initialiseRos(std::vector<std::string> topics){
            if (topics.size()>=2){
                initialiseRobotPublisher(topics[0]);
                initialiseRobotSubscriber(topics[1]);
            }
            if ((this->mode=="recorder")&&(topics.size()>=3)){
                Eigen::Vector3d reading;
                int pos=1;
                this->source_file_path.str("");
                this->source_file_path<<topics[2];
                std::cout<<this->source_file_path.str().c_str()<<std::endl;
                std::ifstream source_file(this->source_file_path.str().c_str());
                source_file>>reading[0];
                while(source_file.get()!=EOF){
                    source_file>>reading[pos];
                    pos = (pos+1) %3;
                    if (pos==0){
                        this->source_control_reference.push_back(reading);
                    }

                }
                std::cout<<this->source_control_reference.size()<<std::endl;
                this->inverse_control_reference.resize(this->source_control_reference.size());
                for (int i=0;i<this->source_control_reference.size();i++){
                   this->inverse_control_reference[i]=this->source_control_reference[this->source_control_reference.size()-i-1];
                }
            }
        }

        void SS_offline_control::robot_callback(const sensor_msgs::JointState &state){
            for (int i=0;i<6;i++){
                this->robot_joint_state[i]=state.position[i];
            }
	        this->robot_ef_state = this->robot_ik->fk_eigen(this->robot_joint_state);
            this->reading = true;
        }

        void SS_offline_control::control_callback(const sensor_msgs::JointState &state){
            ;
        }


        void SS_offline_control::userKeyboardEventControl(){
            char accept;
            std::string name;
            while(this->isGoing){
                name.clear();
                std::cout<<"print the name of the file where the trajectory will be saved"<<std::endl;
                std::cout<<"it will be created in: "<<getSaveFilePath().c_str()<<std::endl;
                std::cin >> name;
                while (!this->reading) std::cout<<"waiting for joint message to come"<<std::endl;
                this->save_file_path << name;
                std::cout<<"do you want to use the force sensor? /type 'y' to accept"<<std::endl;
                std::cin >> accept;
                if (accept !='y'){
                    this->advanced_control = false;
                    std::cout << "press enter to start" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    this->state = INITIALIZING;
                    std::cout << "press enter to stop" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    this->state = SETTING;
                } 
                else{
                    this->advanced_control = true;
                    std::cout << "press enter to start" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    this->state = START;
                    std::cout<<"system_started"<<std::endl;
                    while (this->state!=SS_offline_control::State::WAITING);
                    std::cout << "liquid container detected" << std::endl;
                    std::cout << "system stalled, you can move it into home position"<<std::endl;
                    std::cout << "press enter to start" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    this->homing_complete=true;
                    std::cout << "press enter to stop" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    if (this->state == FILTERING)
                        this->state = SETTING;
                    else 
                        this->state = STOP;
                }
                std::cout<<"waiting for the trajectory to ultimate"<<std::endl;
                while(!(this->state==STOP));
                std::cout << "do you want to record another trajectory? (type 'y' to accept)" << std::endl;
                std::cin >> accept;
                if (accept!='y') this->isGoing = false;
            }
        }

        void SS_offline_control::initialiseRobotTrajectory(){
            this->trajectory_offset = this->source_control_reference[0] - this->robot_ef_state;
            this->loop_counter = 0;
            this->robot_ik->reset();
        }

        void SS_offline_control::initialiseRobotFilteredTrajectory(){
            Eigen::Vector3d temp_state ;
            std::vector<double> extrapolated_parameters;
            std::vector<double> evaluated_parameters;
            if (this->advanced_control){
                extrapolated_parameters=this->tool->extrapolateLiquidParameters();
                std::cout<<"found liquid parameters: "<<extrapolated_parameters[0]<<" "<<extrapolated_parameters[2]<<std::endl;
                evaluated_parameters=this->tool->getFilterParameters();
                for (int i=0;i<3;i++){
                    if (fabs(extrapolated_parameters[i]-evaluated_parameters[i])>0.1)
                        extrapolated_parameters[i]=evaluated_parameters[i];
                }
                this->ss_filter->setFilterParameters(extrapolated_parameters);
                this->ss_filter->setFilterParameters(0.3908,0.002,-0.4044);
                
                this->robot_ik->setToolOffsets(this->tool->getToolOffsetReverse());
                //this->robot_ik->setToolOffsets(Eigen::Vector3d(127.5,0,30));
                this->ss_filter->setSafetyOffset(this->tool->getToolOffsetReverse()/1000);
                std::cout<<"tool_offset "<<this->tool->getToolOffsetReverse()<<std::endl;
            }
            this->robot_ik->reset();
            this->trajectory_check=this->robot_joint_state;
            temp_state = this->robot_ik->fk_eigen(this->robot_joint_state);
            this->ss_filter->setFilterInitialState(temp_state);
            temp_state=this->source_control_reference[0]-temp_state;
            this->ss_filter->setFilterPositionOffset(temp_state);
            this->loop_counter = 0;
            this->save_file.open(this->save_file_path.str().c_str(),std::ofstream::out);
        }

        void SS_offline_control::evaluateUnfilteredReference(){
            KDL::Frame target_frame;
            Eigen::Vector3d temp = this->source_control_reference[0] - this->trajectory_offset;
            target_frame.p=KDL::Vector(1000*temp[0],1000*temp[1],1000*temp[2]);
            target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*temp[1],1000*temp[0]));
            this->robot_ik->ik_neg5(target_frame,temp,false,this->trajectory_reference);
        }

        void SS_offline_control::evaluateNewTrajectoryReference(bool angular_compensation,bool stabilization){
            KDL::Frame target_frame;
            if (stabilization){
                this->filtered_reference = this->ss_filter->evaluateOnlineSingleFilterStep(this->source_control_reference[this->index]);
                this->index++;
            }
            else
                this->filtered_reference = this->ss_filter->evaluateOnlineSingleFilterStep(this->reference_stabilizer);
            writeToFile(save_file,this->source_control_reference[this->index],this->filtered_reference[0],this->filtered_reference[1]);
            target_frame.p=KDL::Vector(1000*this->filtered_reference[0][0],1000*this->filtered_reference[0][1],1000*this->filtered_reference[0][2]);
            target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*this->filtered_reference[0][1],1000*this->filtered_reference[0][0]));
            this->robot_ik->ik_neg5(target_frame,this->filtered_reference[1],angular_compensation,this->trajectory_reference);
        }

        bool SS_offline_control::ikCheck(){
            if (this->trajectory_check.empty()){
                for (int i=0;i<6;i++){
                    this->trajectory_check.push_back(this->trajectory_reference[i]);
                }
                return true;
            }
            else{
                for(int i=0;i<6;i++){
                    if(fabs(this->trajectory_check[i]-this->trajectory_reference[i])>0.2) {
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

        int SS_offline_control::publishRobotReference(){
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

        int SS_offline_control::spin(){
            int vedo=0;
            ros::Rate r(500);
            while(ros::ok()){
                
                switch (this->state){

                    case READING:     if (this->reading) this->state = START;
                                      break;

                    case START:       initialiseRobotTrajectory();
                                        this->state = RUNNING;
                                        break;
                    
                    case RUNNING:     evaluateUnfilteredReference();
                                        //if(publishRobotReference()==-1) return -1;
                                        if (this->tool->getSensorState() == Liquid_handler::Sensor_state::MEASURING)
                                            this->state = CHECKING;
                                        else if (this->tool->getSensorState() == Liquid_handler::Sensor_state::PRESENT)
                                            this->state = INITIALIZING;
                                        break;

                    case CHECKING:    if (this->tool->getSensorState() == Liquid_handler::Sensor_state::READY)
                                            this->state = START;
                                        else if (this->tool->getSensorState() == Liquid_handler::Sensor_state::PRESENT)
                                            this->state = WAITING;
                                        break;

                    case INITIALIZING:  initialiseRobotFilteredTrajectory();
                                        this->state = FILTERING;
                                        this->homing_complete = false;
                                        break;

                    case WAITING:       if(this->homing_complete)
                                            this->state = INITIALIZING;
                                        
                                        break;




                    case FILTERING:     evaluateNewTrajectoryReference(true,true);
                                        if(publishRobotReference()==-1) return -1;
                                        if (this->advanced_control)
                                            if (this->tool->getSensorState() == Liquid_handler::Sensor_state::READY)
                                                this->state = START;
                                        /*if (this->index==this->source_control_reference.size()){
                                            this->state=SETTING;
                                        }*/
                                        if (this->index==this->source_control_reference.size()){
                                            if (this->doppio){
                                                this->index=0;
                                                this->state = STALLING;
                                            }
                                            else
                                                this->state=SETTING;
                                        }


                                        break;

                    case STALLING:      this->index ++;
                                        if (this->index=3000){
                                            this->source_control_reference = this->inverse_control_reference;
                                            this->state = FILTERING;
                                            this->doppio = false;
                                        }


                    case SETTING:     if (this->loop_counter==0){
                                            this->reference_stabilizer=this->source_control_reference[this->index-1];
                                        }
                                        evaluateNewTrajectoryReference(true,false);
                                        if(publishRobotReference()==-1) return -1;
                                            this->loop_counter++;
                                        if(this->loop_counter==ss_filter->getFilterSampleNumber()){
                                            this->save_file.close();
                                            this->state = STOP;
                                            this->trajectory_check.clear();
                                        }
                                        break;

                    case STOP:        if (this->save_file.is_open());
                                            this->save_file.close();
                                        this->trajectory_check.clear();
                                        this->loop_counter = 0;
                                        break;
                }

                ros::spinOnce();
                r.sleep();
            }
            return 0;
        }


        std::thread SS_offline_control::inputThread() {
            return std::thread([=] { userKeyboardEventControl(); });
        }

        void SS_offline_control::writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out){
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

        void SS_offline_control::setSaveFilePath(std::string filepath){
            this->save_file_path.str("");
            this->save_file_path << filepath;
        }

        void SS_offline_control::setSourceTrajectoryFilePath(std::string filepath){
            this->source_file_path.str("");
            this->source_file_path << filepath;
        }

        void SS_offline_control::setSSFilterParameters(std::vector<double> parameters){
            this->ss_filter->setFilterParameters(parameters);
            this->loop_counter = this->ss_filter->getFilterSampleNumber();
        }

        void SS_offline_control::setRobotIkOffsets(Eigen::Vector3d offsets){
            this->robot_ik->setToolOffsets(offsets);
        }


        std::string SS_offline_control::getSaveFilePath(){
            return this->save_file_path.str();
        }

}
		