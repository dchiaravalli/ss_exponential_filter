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
#include "Liquid_handler.h"



namespace ss_exponential_filter  {
	
    	Liquid_handler::Liquid_handler(ros::NodeHandle &n,std::string force_topic,int sampling_number,bool reverse):nh(n) {
            this->posefinder = new ss_exponential_filter::Tool_posefinder(0);
            this->evaluator = new ss_exponential_filter::SS_parameter_evaluator(0.002,0.047);
            this->get_pose_service = this->nh.advertiseService("get_pose_service",&Liquid_handler::service_callback,this);
            this->reverse = reverse;
            initialiseForceSubscriber(force_topic);
            initialiseModelSignalClient("/model_signal");
            resetMeanMeasurements();
            resetMeanReferences();
            this->sampling_number = sampling_number;
            this->isReady = false;
            this->enable_finder=false;
            this->hasLiquid = false;
            this->mean_counter = 0;
            this->stabilizer_wait = 0;
            this->periodic_signal.resize(this->sampling_number);
            this->state = INITIALIZE;

    	}

    	Liquid_handler::~Liquid_handler() {
    	}

        void Liquid_handler::initialiseForceSubscriber(std::string force_subscriber_topic){
            if (this->reverse){
                this->force_sub=this->nh.subscribe(force_subscriber_topic.c_str(), 1,&Liquid_handler::force_callback_reverse,this);
            }
            else{
                this->force_sub=this->nh.subscribe(force_subscriber_topic.c_str(), 1,&Liquid_handler::force_callback,this);
            }
        }

        void Liquid_handler::initialiseModelSignalClient(std::string model_signal_topic){
            this->model_signal_client=this->nh.serviceClient<modelsignal::modelNoiseSignal>(model_signal_topic.c_str());
        }

        void Liquid_handler::force_callback(const geometry_msgs::Twist &force){
            setNewSensorInput(force);
        }

        void Liquid_handler::force_callback_reverse(const geometry_msgs::Twist &force){
            setNewSensorInputReversed(force);
        }

        bool Liquid_handler::service_callback(ss_exponential_filter::force_object_pose::Request &req,ss_exponential_filter::force_object_pose::Response &res){
            if (this->state == PRESENT){

                this->state = MEASURING;

                return true;
            }
            return false;
        }

        void Liquid_handler::setNewSensorInput(geometry_msgs::Twist sensor){
            switch (this->state){
                
                case INITIALIZE:  this->mean_force_min[0]+=sensor.linear.x;
                                    this->mean_force_min[1]+=sensor.linear.y;
                                    this->mean_force_min[2]+=sensor.linear.z;
                                    this->mean_torque_min[0]+=-sensor.angular.x;
                                    this->mean_torque_min[1]+=-sensor.angular.y;
                                    this->mean_torque_min[2]+=sensor.angular.z;
                                    this->mean_counter++;
                                    if (mean_counter==1000){
                                        this->mean_force_min=this->mean_force_min/this->mean_counter;
                                        this->mean_torque_min=this->mean_torque_min/this->mean_counter;
                                        this->mean_counter=0;
                                        this->state = READY;
                                    }
                                    break;

                case READY:         if (this->enable_finder){
                                    if (fabs(sensor.linear.z-mean_force_min[2])>2){
                                        this->state = STABILIZING;
                                    }
                                    }   
                                    break;

                case STABILIZING:  this->stabilizer_wait = this->stabilizer_wait + 1;
                                    if (this->stabilizer_wait == 500){
                                        this->stabilizer_wait = 0;
                                        this->state = MEASURING;
                                    }
                                    break;

                case MEASURING:   this->mean_force_max[0]+=sensor.linear.x-this->mean_force_min[0];
                                    this->mean_force_max[1]+=sensor.linear.y-this->mean_force_min[1];
                                    this->mean_force_max[2]+=fabs(sensor.linear.z-this->mean_force_min[2]);
                                    this->mean_torque_max[0]+=-sensor.angular.x-this->mean_torque_min[0];
                                    this->mean_torque_max[1]+=-sensor.angular.y-this->mean_torque_min[1];
                                    this->mean_torque_max[2]+=sensor.angular.z-this->mean_torque_min[2];
                                    this->periodic_signal[mean_counter]=sensor.linear.y;
                                    this->mean_counter++;
                                    if (this->mean_counter==this->sampling_number){
                                        this->mean_force_max=this->mean_force_max/this->mean_counter;
                                        this->mean_torque_max=this->mean_torque_max/this->mean_counter;
                                        this->mean_counter=0;
                                        this->srv.request.signal=this->periodic_signal;                                     
                    
                                        if (fabs(this->mean_force_max[2])>2)
                                            this->state = PRESENT;
                                        else{
                                            this->state = READY;
                                            resetMeanMeasurements();
                                        }
                                    }     
                                    break;

                case PRESENT:     /*if (fabs(sensor.linear.z+this->mean_force_max[2]-this->mean_force_min[2])>2){
                                        this->state = READY;
                                        resetMeanMeasurements();
                                    }*/
                                    break;

                default:            ;
            }
        }

        void Liquid_handler::setNewSensorInputReversed(geometry_msgs::Twist sensor){
            static int check = 0;
            check++;
            //std::cout<<check<<std::endl;
            switch (this->state){
                case INITIALIZE:    //std::cout<<"initialise"<<std::endl;
                                    this->mean_force_min[0]+=sensor.linear.x;
                                    this->mean_force_min[1]+=sensor.linear.y;
                                    this->mean_force_min[2]+=sensor.linear.z;
                                    this->mean_torque_min[0]+=-sensor.angular.x;
                                    this->mean_torque_min[1]+=sensor.angular.y;
                                    this->mean_torque_min[2]+=sensor.angular.z;
                                    this->mean_counter++;
                                    if (mean_counter==1000){
                                        this->mean_force_min=this->mean_force_min/this->mean_counter;
                                        this->mean_torque_min=this->mean_torque_min/this->mean_counter;
                                        this->mean_counter=0;
                                        this->state = READY;
                                        std::cout<<"go to ready"<<std::endl;
                                    }
                                    break;

                case READY:         //std::cout<<"ready"<<std::endl;
                                    if (this->enable_finder){
                                    if (fabs(sensor.linear.z-mean_force_min[2])>0){
                                        this->state = STABILIZING;
                                        std::cout<<"go to stabilizing"<<std::endl;
                                    }
                                    }   
                                    break;

                case STABILIZING:   //std::cout<<"stabilizing"<<std::endl;
                                    this->stabilizer_wait = this->stabilizer_wait + 1;
                                    if (this->stabilizer_wait == 1000){
                                        this->stabilizer_wait = 0;
                                        this->state = MEASURING;
                                        std::cout<<"go to measuring"<<std::endl;
                                    }
                                    break;

                case MEASURING:     //std::cout<<"measuring"<<std::endl;
                                    this->mean_force_max[0]+=sensor.linear.x-this->mean_force_min[0];
                                    this->mean_force_max[1]+=sensor.linear.y-this->mean_force_min[1];
                                    this->mean_force_max[2]+=fabs(sensor.linear.z-this->mean_force_min[2]);
                                    this->mean_torque_max[0]+=-sensor.angular.x-this->mean_torque_min[0];
                                    this->mean_torque_max[1]+=sensor.angular.y-this->mean_torque_min[1];
                                    this->mean_torque_max[2]+=sensor.angular.z-this->mean_torque_min[2];
                                    this->periodic_signal[mean_counter]=sensor.linear.y;
                                    this->mean_counter++;
                                    if (this->mean_counter==this->sampling_number){
                                        std::cout<<"reversed"<<std::endl;
                                        this->mean_force_max=this->mean_force_max/this->mean_counter;
                                        this->mean_torque_max=this->mean_torque_max/this->mean_counter;
                                        this->mean_counter=0;
                                        this->srv.request.signal=this->periodic_signal;                                     
                    
                                        if (fabs(this->mean_force_max[2])>0){
                                            this->state = PRESENT;
                                            std::cout<<"ready"<<std::endl;
                                        }
                                        else{
                                            this->state = READY;
                                            resetMeanMeasurements();
                                        }
                                    }     
                                    break;

                case PRESENT:       std::cout<<"measurement"<<std::endl;
                                    std::cout<<this->getToolOffsetReverse()<<std::endl;
                                    //std::cout<<"present"<<std::endl;
                                    /*if (fabs(sensor.linear.z+this->mean_force_max[2]-this->mean_force_min[2])>2){
                                        this->state = READY;
                                        resetMeanMeasurements();
                                    }*/
                                    break;

                default:            ;
            }
        }

        void Liquid_handler::evaluateLiquidParameters(){
            this->posefinder->setSensorRead(this->mean_force_max,this->mean_torque_max);
            this->evaluator->setLiquidMass(this->posefinder->getFluidMass(0.2));
        }

        std::vector<double> Liquid_handler::extrapolateLiquidParameters(){
            std::vector<double> param(3);
            std::fill(param.begin(),param.end(),0.0);
            if(this->model_signal_client.call(srv))
            {
                param[0]=srv.response.parameters[1];
                param[1]=0.002;
                param[2]=srv.response.parameters[0];
            }
            else
                std::cout<<"epic fail"<<std::endl;
            return param;
        }

        std::vector<double> Liquid_handler::getFilterParameters(){
            evaluateLiquidParameters();
            return this->evaluator->getFilterParameters();
        }

        Eigen::Vector3d Liquid_handler::getToolOffset(){
            evaluateLiquidParameters();
            return 1000*(this->posefinder->getToolPose(0.005,0.0335)+Eigen::Vector3d(0,0,this->evaluator->getFluidHeight()));
        //    return Eigen::Vector3d(0,0,this->evaluator->getFluidHeight());
        }

        Eigen::Vector3d Liquid_handler::getToolOffsetReverse(){
            evaluateLiquidParameters();
            return 1000*(this->posefinder->getToolPose(0,-0.16)+Eigen::Vector3d(0,0,this->evaluator->getFluidHeight()));
        }

        Eigen::Vector3d Liquid_handler::getToolRigidObjectReverse(){
            evaluateLiquidParameters();
            return 1000*(this->posefinder->getToolPose(0,-0.17));
        }

        Eigen::Vector3d Liquid_handler::getFixedPose(){
            evaluateLiquidParameters();
            return 1000*(this->posefinder->getToolPose(0.005,0.0335)+Eigen::Vector3d(0,0,0.169-0.0168));
        }

        void Liquid_handler::resetMeanMeasurements(){
            mean_force_max = Eigen::Vector3d(0,0,0);
            mean_torque_max = Eigen::Vector3d(0,0,0);
        }

        void Liquid_handler::resetMeanReferences(){
            mean_force_min = Eigen::Vector3d(0,0,0);
            mean_torque_min = Eigen::Vector3d(0,0,0);  
            this->isReady = false;         
        }

        void Liquid_handler::setSamplingNumber(int sampling_number){
            this->sampling_number = sampling_number;
        }

        void Liquid_handler::setEnableFinder(){
            this->enable_finder=true;
        }

        Liquid_handler::Sensor_state Liquid_handler::getSensorState(){
            return this->state;
        }

        Eigen::Vector3d Liquid_handler::getMinForceMeasure(){
            return this->mean_force_min;
        }

        Eigen::Vector3d Liquid_handler::getMaxForceMeasure(){
            return this->mean_force_max;
        }

        Eigen::Vector3d Liquid_handler::getMinTorqueMeasure(){
            return this->mean_torque_min;
        }

        Eigen::Vector3d Liquid_handler::getMaxTorqueMeasure(){
            return this->mean_torque_max;
        }

        std::vector<Eigen::Vector3d> Liquid_handler::getMinMeasure(){
            std::vector<Eigen::Vector3d> measure;
            measure.push_back(this->mean_force_min);
            measure.push_back(this->mean_torque_min);
        }

        std::vector<Eigen::Vector3d> Liquid_handler::getMaxMeasure(){
            std::vector<Eigen::Vector3d> measure;
            measure.push_back(this->mean_force_max);
            measure.push_back(this->mean_torque_max);
        }
}
		