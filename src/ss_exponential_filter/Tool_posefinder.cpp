/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Tool_posefinder.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Jen 17, 2018, 4:05 PM
 */


#include <stdio.h>
#include <math.h>
#include <iostream>
#include "Tool_posefinder.h"

namespace ss_exponential_filter  {
	

	
    	Tool_posefinder::Tool_posefinder(double sensor_offset) {    
            setSensorRead(Eigen::Vector3d(0,0,1),Eigen::Vector3d(0,0,0));  
            this->sensor_offset=sensor_offset; 
    	}

    	Tool_posefinder::~Tool_posefinder() {
    	}

        void Tool_posefinder::setSensorForceRead(Eigen::Vector3d forces){
            this->forces=forces;
        }

        void Tool_posefinder::setSensorTorqueRead(Eigen::Vector3d torques){
            this->torques=torques;
        }

        void Tool_posefinder::setSensorOffset(double sensor_offset){
            this->sensor_offset=sensor_offset;
        }

        void Tool_posefinder::setSensorRead(Eigen::Vector3d forces, Eigen::Vector3d torques){
            this->torques=torques;
            this->forces=forces;
            this->z_force=Eigen::Vector3d(0,0,this->forces[2]); 
        }       

        double Tool_posefinder::getFluidMass(double offset){
            return this->forces[2]/9.8067-offset;
        }

        Eigen::Vector3d Tool_posefinder::getToolPose(double bessel_thickness,double sensor_to_joint){
            Eigen::Vector3d final_pose;
            final_pose=1/pow(this->forces[2],2)*(this->z_force.cross(this->torques)+this->z_force*this->sensor_offset*this->forces[2]);
            final_pose[2]=final_pose[2]+bessel_thickness+sensor_to_joint;
            return final_pose;
        }




}
