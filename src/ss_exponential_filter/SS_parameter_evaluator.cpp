/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SS_parameter_evaluator.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Jen 17, 2018, 4:05 PM
 */


#include <stdio.h>
#include <math.h>
#include <iostream>
#include "SS_parameter_evaluator.h"

namespace ss_exponential_filter  {
	

	
    	SS_parameter_evaluator::SS_parameter_evaluator(double sampling_time,double cylinder_radius, double fluid_mass) {    
            this->fluid_mass=fluid_mass;
            this->sampling_time=sampling_time;
            this->cylinder_radius=cylinder_radius;                         
            this->fluid_specific_gravity=1000;                    
            this->kinematic_viscosity=0.000001004;        
            this->earth_gravity=9.80665;                                                
            this->bessel_ki=1.8412;     
            evaluateFluidParameters();                            
    	}

    	SS_parameter_evaluator::~SS_parameter_evaluator() {
    	}

        void SS_parameter_evaluator::setLiquidMass(double mass){
            this->fluid_mass=mass;
            evaluateFluidParameters(); 
        }

        void SS_parameter_evaluator::setCylinderRadius(double radius){
            this->cylinder_radius=radius;
            evaluateFluidParameters(); 
        }

        void SS_parameter_evaluator::setSamplingTime(double sampling_time){
            this->sampling_time=sampling_time;
        }

        void SS_parameter_evaluator::setSystemParameters(double sampling_time, double radius, double mass){
            this->sampling_time=sampling_time;
            this->cylinder_radius=radius;
            this->fluid_mass=mass;
            evaluateFluidParameters(); 

        }

        void SS_parameter_evaluator::evaluateFluidParameters(){
            this->fluid_volume=this->fluid_mass/this->fluid_specific_gravity;
            this->fluid_height=this->fluid_volume/(M_PI*pow(this->cylinder_radius,2));  
            this->pendulum_frequency=sqrt((this->earth_gravity*this->bessel_ki/this->cylinder_radius)*tanh(this->fluid_height*this->bessel_ki/this->cylinder_radius));
            this->L1=2*this->cylinder_radius/(this->bessel_ki*sinh(this->fluid_height*this->bessel_ki/this->cylinder_radius));
            this->rei=sqrt(this->kinematic_viscosity/(pow(this->cylinder_radius,1.5)*pow(this->earth_gravity,0.5)));
            this->delta=(2.89/M_PI)*this->rei*(1+(0.318/sinh((1.84*this->fluid_height)/this->cylinder_radius))*(1+((1-this->fluid_height/this->cylinder_radius)/cosh((1.84*this->fluid_height)/this->cylinder_radius))));
            this->d1=this->delta;
        }

        std::vector<double> SS_parameter_evaluator::getFilterParameters(){
            std::vector<double> parameters;
            parameters.resize(3);
            parameters[0]=(3*M_PI/(this->pendulum_frequency*sqrt(1-pow(this->d1,2))));
            parameters[1]=this->sampling_time;
            parameters[2]=(-this->d1*this->pendulum_frequency);

            return parameters;
        }

        std::vector<double> SS_parameter_evaluator::getLiquidParameters(){
            std::vector<double> parameters;
            parameters.resize(2);
            parameters[0]=this->pendulum_frequency;
            parameters[1]=this->d1;

            return parameters;
        }



        std::vector<double> SS_parameter_evaluator::getFilterParameters(double mass){
            setLiquidMass(mass);
            std::vector<double> parameters;
            parameters.push_back(3*M_PI/(this->pendulum_frequency*sqrt(1-pow(this->d1,2))));
            parameters.push_back(this->sampling_time);
            parameters.push_back(-this->d1*this->pendulum_frequency);
            return parameters;
        }

        double SS_parameter_evaluator::getFluidHeight(){
            if (this->fluid_height>this->cylinder_radius)
                return this->fluid_height-this->L1;
            else
                return this->fluid_height;
        }


}
