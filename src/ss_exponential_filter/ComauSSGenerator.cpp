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

#include "ComauSSGenerator.h"
#include <stdio.h>
#include <math.h>
#include <iostream>


namespace ss_exponential_filter {
	

	
    	ComauSSGenerator::ComauSSGenerator(int filter_order,double filterperiod,double filterTs) {
			this->iteration_count=0;
			this->input_point=1;
			this->filter_order=filter_order;
			for (int i=0;i<this->filter_order;i++)
				filter.push_back(FIRFilter(filterperiod,filterTs));
    	}

    	ComauSSGenerator::~ComauSSGenerator() {
    	}

		void ComauSSGenerator::setNewViaPoints(std::vector<Eigen::Vector3d> via_points){
			this->via_points=via_points;
		}


		/**
		* this method converts a via point vector into the corresponding control point vector for spline generation
		* 
		*/
		void ComauSSGenerator::viaPointsToControlPoints(){

			//variable declaration
			int n=this->via_points.size()-2;
			std::vector<Eigen::Vector3d> temporary_points=this->via_points;
			std::vector<float> main_diagonal(n);
		
			//variable initialisation
			if (n<2)
				std::cout<<"via points number too small"<<std::endl;
			//prepare d vector
			temporary_points.erase(temporary_points.begin()+n+1);
			temporary_points.erase(temporary_points.begin());

			for(int i=0;i<temporary_points.size();i++)
			{
				temporary_points[i]=temporary_points[i]*6;
			}
			this->control_points.resize(n+2);

			temporary_points[0]=temporary_points[0]-this->via_points[0];
			temporary_points[n-1]=temporary_points[n-1]-this->via_points[n+1];
			main_diagonal[0]=4;

			//forward elimination
			for (int k=1;k<n;k++)
			{
				main_diagonal[k]=4-0.25;
				temporary_points[k]=temporary_points[k]-temporary_points[k-1]*0.25;
			}
			
			//backward_substitution
			this->control_points[n-1]=temporary_points[n-1]/main_diagonal[n-1];
			for (int k=n-2;k>=0;k--)
				this->control_points[k]=(temporary_points[k]-this->control_points[k+1])/main_diagonal[k];
			//vector shift for first and last points
			for (int i=n-1;i>=0;i--)
				this->control_points[i+1]=this->control_points[i];
			this->control_points[0]=this->via_points[0];		
			this->control_points[n+1]=this->via_points[n+1];

			//add two additional points required by the three filters to evaluate the whole trajectory
			this->control_points.push_back(this->control_points[n+1]);
			this->control_points.push_back(this->control_points[n+1]);
		}


		void ComauSSGenerator::startNewSplineTrajectory(){
			this->iteration_count=0;
			this->input_point=1;
		}


		/**
		* this function creates a Bspline trajectory starting from a vector of via points
		*/
		Eigen::Vector3d ComauSSGenerator::generateBSplineTrajectoryInput(bool& up){

			//Control_point conversion
			this->viaPointsToControlPoints();


			this->filter[0].addNewInput(this->control_points[this->input_point]);
			this->filter[1].addNewInput(this->filter[0].evaluateFilterOutput());
			this->filter[2].addNewInput(this->filter[1].evaluateFilterOutput());
			if (this->iteration_count%this->filter[0].getFilterCellNumber()==this->filter[0].getFilterCellNumber()-1)
			{
				if (this->input_point<3)
					this->input_point++;
				else 	up=true;
			}
			else up=false;
			this->iteration_count++;
			return this->filter[2].evaluateFilterOutput();

		}

		/**
		* this function creates a Bspline trajectory starting from a vector of via points
		*/
		Eigen::Vector3d ComauSSGenerator::generateSimplifiedBSplineTrajectoryInput(bool& up){
			this->filter[0].addNewInput(this->via_points[this->via_points.size()-1]);
			for (int i=1;i<this->filter_order;i++){
				this->filter[i].addNewInput(this->filter[i-1].evaluateFilterOutput());
			}
			if (this->iteration_count%this->filter[0].getFilterCellNumber()==this->filter[0].getFilterCellNumber()-1)
			{
				up=true;
			}
			else up=false;
			this->iteration_count++;
			return this->filter[this->filter_order-1].evaluateFilterOutput();

		}

		void ComauSSGenerator::initialiseFilters(Eigen::Vector3d initial_value){
			for (int i=0;i<3;i++)
			{
				this->filter.at(i).initialiseFilter(initial_value);
			}
		}
		/**
		* this function creates a Bspline trajectory starting from a vector of via points
		*/
		std::vector<Eigen::Vector3d> ComauSSGenerator::generateBSpline(){
			int cell_number=this->filter[0].getFilterCellNumber();
			this->spline_trajectory.clear();

			//Filter state initialisation
			for (int i=0;i<3;i++)
			{
				this->filter.at(i).initialiseFilter(this->via_points.at(0));
			}

			//Control_point conversion
			this->viaPointsToControlPoints();

			//Trajectory evaluation
			for (int i=0;i<this->control_points.size();i++)
			{
				for (int j=0;j<cell_number;j++)
				{
					this->filter[0].addNewInput(this->control_points[i]);
					this->filter[1].addNewInput(this->filter[0].evaluateFilterOutput());
					this->filter[2].addNewInput(this->filter[1].evaluateFilterOutput());
					this->spline_trajectory.push_back(this->filter[2].evaluateFilterOutput());
				}	
			}
			return this->spline_trajectory;
		}


		
		/**
		* this function creates a Bspline trajectory starting from a vector of via points
		*/
		std::vector<Eigen::Vector3d> ComauSSGenerator::generateSimplifiedBSpline(){
			int cell_number=this->filter[0].getFilterCellNumber();
			this->spline_trajectory.clear();

			//Filter state initialisation
			for (int i=0;i<this->filter_order;i++)
				this->filter[i].initialiseFilter(this->via_points[0]);

			//Control_point conversion
			for (int i=1;i<this->filter_order;i++)
			this->via_points.push_back(this->via_points[this->via_points.size()-1]);

			//Trajectory evaluation
			for (int i=0;i<this->via_points.size();i++)
			{
				for (int j=0;j<cell_number;j++)
				{
					this->filter[0].addNewInput(this->via_points[i]);
					this->filter[1].addNewInput(this->filter[0].evaluateFilterOutput());
					this->spline_trajectory.push_back(this->filter[1].evaluateFilterOutput());
				}	
			}
			return this->spline_trajectory;
		}
}
