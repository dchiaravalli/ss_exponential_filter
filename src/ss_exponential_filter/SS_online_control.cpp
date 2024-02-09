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
#include "SS_online_control.h"


namespace ss_exponential_filter  {
	
    	SS_online_control::SS_online_control(ros::NodeHandle &n,std::vector<std::string> &topics,std::string mode):nh(n) {
            this->mode=mode;
            this->isGoing=true;
            this->advanced_control = false;
            this->search_liquid = false;
            this->state = STOP;
            this->robot_ik = new ss_exponential_filter::ComauIK();
            this->ss_filter = new ss_exponential_filter::SS_filter();
            if (topics.size()==4)
                this->tool = new ss_exponential_filter::Liquid_handler(n,topics[3],2500,true);
            this->min_workspace_saturation = Eigen::Vector3d(0.6,-0.4,0.6);
            this->max_workspace_saturation = Eigen::Vector3d(1,0.4,1.1);
            this->min_vicon_saturation = Eigen::Vector3d(0,0,0);
            this->max_vicon_saturation = Eigen::Vector3d(0,0,0);
            this->robot_ef_state = Eigen::Vector3d(0.6,0,1.4);
            this->loop_counter = this->ss_filter->getFilterSampleNumber();
            this->trajectory_reference.resize(6);
            this->robot_joint_state.resize(6);
            this->control_reference = Eigen::Vector3d(0,0,0);
            std::fill(this->trajectory_reference.begin(),this->trajectory_reference.end(),0.0);
            std::fill(this->robot_joint_state.begin(),this->robot_joint_state.end(),0.0);
            this->filter_pub = this->nh.advertise<sensor_msgs::JointState>("filter_output", 1, true);
            this->state_machine=&SS_online_control::lol;
            initialiseRos(topics);
    	}

        void SS_online_control::lol(){
            std::cout<<"lol"<<std::endl;
        }

    	SS_online_control::~SS_online_control() {
    	}

        void SS_online_control::initialiseRobotPublisher(std::string &robot_publisher_topic){
            this->robot_pub=this->nh.advertise<sensor_msgs::JointState>(robot_publisher_topic.c_str(), 1, true);
        }

        void SS_online_control::initialiseRobotSubscriber(std::string &robot_subscriber_topic){
            this->robot_sub=this->nh.subscribe(robot_subscriber_topic.c_str(), 1,&SS_online_control::robot_callback,this);
        }

        void SS_online_control::initialiseControlSubscriber(std::string &control_subscriber_topic){
            this->control_sub=this->nh.subscribe(control_subscriber_topic.c_str(), 1,&SS_online_control::control_callback,this);
        }

        void SS_online_control::initialiseRos(std::vector<std::string> topics){
            if (topics.size()>=2){
                initialiseRobotPublisher(topics[0]);
                initialiseRobotSubscriber(topics[1]);
            }
            if ((this->mode!="vicon")&&(topics.size()>=3)){
                initialiseControlSubscriber(topics[2]);
            }
        }

        void SS_online_control::robot_callback(const sensor_msgs::JointState &state){
            for (int i=0;i<6;i++){
                this->robot_joint_state[i]=state.position[i];
            }
	        this->robot_ef_state = this->robot_ik->fk_eigen(this->robot_joint_state);
        }

        void SS_online_control::control_callback(const geometry_msgs::Pose &msg){
            static bool first_msg = true;
            if (first_msg){
                first_msg = false;
                this->control_reference[0] = msg.position.x;
                this->control_reference[1] = msg.position.y;
                this->control_reference[2] = msg.position.z;
            }
            else if (this->controlErrorControl(msg)){
                this->control_reference[0] = msg.position.x;
                this->control_reference[1] = msg.position.y;
                this->control_reference[2] = msg.position.z;
            }

        }

        void SS_online_control::checkVicon(std::string base_ref,std::string new_ref){
            while (!listener.waitForTransform( base_ref.c_str(), new_ref.c_str(),ros::Time(0), ros::Duration(10.0))){
                std::cout<<"no transform published"<<std::endl;
            }  
            std::cout<<"vicon system enabled"<<std::endl; 
            try
            {
            listener.lookupTransform(base_ref.c_str(), new_ref.c_str(), ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
            ROS_ERROR("%s",ex.what());
            }
            this->control_reference[0]=this->transform.getOrigin().x();
            this->control_reference[1]=this->transform.getOrigin().y();
            this->control_reference[2]=this->transform.getOrigin().z();
        }

        void SS_online_control::updateViconReference(std::string &base_ref,std::string &new_ref,float threshold){
            try
            {
                listener.lookupTransform( base_ref.c_str(), new_ref.c_str(), ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
			if (viconErrorControl(threshold)){
				this->control_reference[0]=this->transform.getOrigin().x();
				this->control_reference[1]=this->transform.getOrigin().y();
				this->control_reference[2]=this->transform.getOrigin().z();
            }
            //this->viconSaturation();
        }

        bool SS_online_control::viconErrorControl(float threshold){
            double temp;
            temp = pow(this->control_reference[0]-this->transform.getOrigin().x(),2);
            temp = temp + pow(this->control_reference[1]-this->transform.getOrigin().y(),2);
            temp = temp + pow(this->control_reference[2]-this->transform.getOrigin().z(),2);
            if(sqrt(temp)<threshold)
                return true;
            return false;
        }

        bool SS_online_control::controlErrorControl(geometry_msgs::Pose pose,float threshold){
            double temp;
            temp = pow(this->control_reference[0]-pose.position.x,2);
            temp = temp + pow(this->control_reference[1]-pose.position.y,2);
            temp = temp + pow(this->control_reference[2]-pose.position.z,2);
            if(sqrt(temp)<threshold)
                return true;
            return false;
        }

        void SS_online_control::viconSaturation(){
            bool found = false;
            for(int i=0;i<3;i++){
                if(this->control_reference[i]>this->max_vicon_saturation[i]){
                    //std::cout<<"max_"<<i<<std::endl;
                    this->control_reference[i] = this->max_vicon_saturation[i];
                    found = true;
                }
                if(this->control_reference[i]<this->min_vicon_saturation[i]){
                    //std::cout<<"min_"<<i<<std::endl;
                    this->control_reference[i] = this->min_vicon_saturation[i];
                    found = true;
                }
            }
            if(!found){
                //std::cout<<"no found"<<std::endl;
            }


        }

        void SS_online_control::userKeyboardEventControl(){
            char accept;
            std::string name;
            while(this->isGoing){
                name.clear();
                std::cout<<"print the name of the file where the trajectory will be saved"<<std::endl;
                std::cout<<"it will be created in: "<<getSaveFilePath().c_str()<<std::endl;
                std::cin >> name;
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
                    std::cout << "press enter to search for liquids" << std::endl;
                    std::cin.ignore();
                    std::cin.get();
                    this->search_liquid = true;
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
                if (accept!='y'){
                   this->isGoing = false;
                   ros::shutdown();
                } 
            }
        }

        void SS_online_control::initialiseRobotTrajectory(){
            this->trajectory_offset = this->control_reference - this->robot_ef_state;
            this->setViconSaturation();
            this->loop_counter = 0;
            this->robot_ik->reset();
        }

        void SS_online_control::initialiseRobotFilteredTrajectory(){
            Eigen::Vector3d temp_state ;
            this->setViconSaturation();
            std::vector<double> extrapolated_parameters;
            std::vector<double> evaluated_parameters;
            if (this->advanced_control){
                extrapolated_parameters=this->tool->extrapolateLiquidParameters();
                evaluated_parameters=this->tool->getFilterParameters();
                for (int i=0;i<3;i++){
                    if (fabs(extrapolated_parameters[i]-evaluated_parameters[i])>0.1)
                        extrapolated_parameters[i]=evaluated_parameters[i];
                }
                this->ss_filter->setFilterParameters(extrapolated_parameters);
                this->ss_filter->setFilterParameters(0.48,0.002,-0.10);
                std::cout<<"found liquid parameters: "<<this->tool->extrapolateLiquidParameters()[0]<<" "<<this->tool->extrapolateLiquidParameters()[2]<<std::endl;
                Eigen::Vector3d force_offset = this->tool->getToolOffsetReverse();
                force_offset[2] = -15;
                this->setRobotIkOffsets(force_offset);
                this->ss_filter->setSafetyOffset(this->tool->getToolOffsetReverse()/1000);
                std::cout<<"tool_offset "<<this->tool->getToolOffsetReverse()<<std::endl;
            }
            this->ss_filter->setFilterParameters(0.48,0.002,-0.10);
            this->robot_ik->reset();
            this->trajectory_check=this->robot_joint_state;
            std::cout<<"robot joint states "<<this->robot_joint_state[0]<<" "<<this->robot_joint_state[1]<<" "<<this->robot_joint_state[2]<<" "<<this->robot_joint_state[3]<<" "<<this->robot_joint_state[4]<<" "<<this->robot_joint_state[5]<<std::endl;
            temp_state = this->robot_ik->fk_eigen(this->robot_joint_state);
            std::cout<<"robot new ws position "<<temp_state[0]<<" "<<temp_state[1]<<" "<<temp_state[2]<<" "<<std::endl;
            this->ss_filter->setFilterInitialState(temp_state);
            temp_state=this->control_reference-temp_state;
            this->ss_filter->setFilterPositionOffset(temp_state);
            this->loop_counter = 0;
            this->save_file.open(this->save_file_path.str().c_str(),std::ofstream::out);
        }

        void SS_online_control::evaluateUnfilteredReference(){
            KDL::Frame target_frame;
            Eigen::Vector3d temp = this->control_reference - this->trajectory_offset;
            target_frame.p=KDL::Vector(1000*temp[0],1000*temp[1],1000*temp[2]);
            target_frame.M=KDL::Rotation::RPY(0,0,0);
            //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*temp[1],1000*temp[0]));
            this->robot_ik->ik_neg5(target_frame,temp,false,this->trajectory_reference);
        }

        void SS_online_control::evaluateNewTrajectoryReference(bool angular_compensation,bool stabilization){
            KDL::Frame target_frame;
            if (stabilization)
                this->filtered_reference = this->ss_filter->evaluateOnlineSingleFilterStep(this->control_reference);
            else
                this->filtered_reference = this->ss_filter->evaluateOnlineSingleFilterStep(this->reference_stabilizer);
            writeToFile(save_file,this->control_reference,this->filtered_reference[0],this->filtered_reference[1]);
            target_frame.p=KDL::Vector(1000*this->filtered_reference[0][0],1000*this->filtered_reference[0][1],1000*this->filtered_reference[0][2]);
            //target_frame.M=KDL::Rotation::RPY(0,0,atan2(1000*this->filtered_reference[0][1],1000*this->filtered_reference[0][0]));
            target_frame.M=KDL::Rotation::RPY(0,0,0);
            sensor_msgs::JointState filter_msg;
            filter_msg.position.resize(6);
            for(int i=0;i<3;i++){
                filter_msg.position[i] = this->filtered_reference[0][i];
                filter_msg.position[i+3] = this->filtered_reference[1][i];
            }
            ros::Time tmsg=ros::Time::now();
			filter_msg.header.stamp.sec=tmsg.sec;
			filter_msg.header.stamp.nsec=tmsg.nsec;
            this->filter_pub.publish(filter_msg);
            this->robot_ik->ik_neg5(target_frame,this->filtered_reference[1],angular_compensation,this->trajectory_reference);
        }

        bool SS_online_control::ikCheck(){
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

        int SS_online_control::publishRobotReference(){
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

        int SS_online_control::spin(std::string base_ref,std::string new_ref,float threshold){
            ros::Rate r(500);
            while(ros::ok()){
                //std::cout<<this->state<<std::endl;
                    //updateViconReference(base_ref,new_ref);
                switch (this->state){
                    case START:         //std::cout<<"start"<<std::endl;
                                        initialiseRobotTrajectory();
                                        this->state = RUNNING;
                                        break;
                    
                    case RUNNING:       viconSaturation();
                                        evaluateUnfilteredReference();
                                        //std::cout<<"running"<<std::endl;
                                        if(publishRobotReference()==-1) return -1;
                                        if (this->search_liquid) this->tool->setEnableFinder();
                                        //std::cout<<"sensor_state"<<std::endl;
                                        //std::cout<<this->tool->getSensorState()<<std::endl;
                                        if (this->tool->getSensorState() == Liquid_handler::Sensor_state::MEASURING)
                                            this->state = CHECKING;
                                        else if (this->tool->getSensorState() == Liquid_handler::Sensor_state::PRESENT)
                                            this->state = INITIALIZING;
                                        break;

                    case CHECKING:      //std::cout<<"checking"<<std::endl;
                                        if (this->tool->getSensorState() == Liquid_handler::Sensor_state::READY)
                                            this->state = START;
                                        else if (this->tool->getSensorState() == Liquid_handler::Sensor_state::PRESENT)
                                            this->state = INITIALIZING;
                                        break;

                    case INITIALIZING:  //std::cout<<"initializing"<<std::endl;
                                        initialiseRobotFilteredTrajectory();
                                        this->state = FILTERING;
                                        break;

                    case FILTERING:     //std::cout<<"filtering"<<std::endl;
                                        viconSaturation();
                                        evaluateNewTrajectoryReference(true,true);
                                        if(publishRobotReference()==-1) return -1;
                                        /*if (this->tool->getSensorState() == Liquid_handler::Sensor_state::READY)
                                            this->state = START;*/
                                        break;

                    case SETTING:       //std::cout<<"setting"<<std::endl;
                                        if (this->loop_counter==0){
                                            this->reference_stabilizer=this->control_reference;
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

                    case STOP:          //std::cout<<"stop"<<std::endl;
                                        if (this->save_file.is_open());
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


        std::thread SS_online_control::inputThread() {
            return std::thread([=] { userKeyboardEventControl(); });
        }

        void SS_online_control::writeToFile(std::ofstream & traj_file, Eigen::Vector3d & vect_in, Eigen::Vector3d & vect_out, Eigen::Vector3d &acc_out){
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

        void SS_online_control::setWorkspaceSaturation(Eigen::Vector3d min_saturation,Eigen::Vector3d max_saturation){
            this->min_workspace_saturation = min_saturation;
            this->max_workspace_saturation = max_saturation;
        }

        void SS_online_control::setSaveFilePath(std::string filepath){
            this->save_file_path.str("");
            this->save_file_path << filepath;
        }

        void SS_online_control::setViconSaturation(){
            Eigen::Vector3d max_offset = this->max_workspace_saturation - this->robot_ef_state;
            Eigen::Vector3d min_offset = this->robot_ef_state - this->min_workspace_saturation;
            this->max_vicon_saturation = this->control_reference + max_offset;
            this->min_vicon_saturation = this->control_reference - min_offset;
            std::cout<<"viconsaturation"<<std::endl;
            std::cout<<this->control_reference<<std::endl;
            std::cout<<this->max_vicon_saturation<<std::endl;
            std::cout<<this->min_vicon_saturation<<std::endl;
        }


        void SS_online_control::setSSFilterParameters(std::vector<double> parameters){
            this->ss_filter->setFilterParameters(parameters);
            this->loop_counter = this->ss_filter->getFilterSampleNumber();
        }

        void SS_online_control::setRobotIkOffsets(Eigen::Vector3d offsets){
            this->robot_ik->setToolOffsets(offsets);
        }


        std::string SS_online_control::getSaveFilePath(){
            return this->save_file_path.str();
        }

        Eigen::Vector3d SS_online_control::getMinWorkspaceSaturation(){
            return this->min_workspace_saturation;
        }

        Eigen::Vector3d SS_online_control::getMaxWorkspaceSaturation(){
            return this->max_workspace_saturation;
        }

}
		