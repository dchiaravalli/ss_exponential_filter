/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Agent.cpp
 * Author: Davide Chiaravalli, Federico Califano
 * 
 * Created on Sep 8, 2016, 2:05 PM
 */

#include "FIRFilter.h"
#include <stdio.h>
#include <math.h>
#include <iostream>

namespace ss_exponential_filter {
	

	
    	FIRFilter::FIRFilter(double period,double Ts) {
			this->period=period;						//time period of the spline
			this->sampling_time=Ts;				//sampling time of the filters used to evaluate the spline
			this->cell_number=(int)(this->period/this->sampling_time);			//number of cells of the filters
            this->next_input_position=0;
            this->state.resize(this->cell_number);
            this->initialiseFilter(Eigen::Vector3d(0,0,0));
    	}

    	FIRFilter::~FIRFilter() {
    	}

		void FIRFilter::initialiseFilter(Eigen::Vector3d initial_value){
            for(int i=0;i<this->cell_number;i++)
                    this->state[i]=initial_value;
            this->resetInputPosition();
        }

        void FIRFilter::setFilterParameters(double period, double Ts){
            this->period=period;						
			this->sampling_time=Ts;
            std::cout<<period<<std::endl;
            std::cout<<Ts<<std::endl;
            this->cell_number=(int)(this->period/this->sampling_time);
            std::cout<<"celle"<<this->cell_number<<std::endl;
            std::vector<Eigen::Vector3d> temp;
            temp.resize(this->cell_number);
            this->state=temp;
            //this->initialiseFilter(Eigen::Vector3d(0,0,0));
        }
    	
        void FIRFilter::resetInputPosition(){
            this->next_input_position=0; 
        }

        void FIRFilter::addNewInput(Eigen::Vector3d input){
            this->state[next_input_position]=input;
            this->next_input_position=(this->next_input_position+1)%this->cell_number;
        }

        Eigen::Vector3d FIRFilter::evaluateFilterOutput(){
            Eigen::Vector3d sum(0,0,0);
            for (int i=0;i<this->cell_number;i++)
                sum=sum+this->state[i];
            return sum/this->cell_number;
        }

        int FIRFilter::getFilterCellNumber(){
            return this->cell_number;
        }
}
