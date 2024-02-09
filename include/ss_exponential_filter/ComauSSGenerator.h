/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   YouBot.h
 * Author: Davide Chiaravalli, Federico Califano
 *
 * Created on Sep 8, 2016, 2:05 PM
 */

#ifndef COMAUSSGENERATOR_H
#define COMAUSSGENERATOR_H

#include <vector>
#include <Eigen/Dense>
#include "FIRFilter.h"


namespace ss_exponential_filter {

    class ComauSSGenerator {
    public:
	ComauSSGenerator(int filter_order,double filterperiod=1,double filterTs=0.001);			//Class Constructor #1
    virtual ~ComauSSGenerator();			//Class Destructor


        //Methods

	//default variable handler functions
	void setNewViaPoints(std::vector<Eigen::Vector3d> via_points);


	//via points to control point conversion
	void viaPointsToControlPoints();

	//Bspline trajectory single next reference generation
	Eigen::Vector3d generateBSplineTrajectoryInput(bool& up);
	Eigen::Vector3d generateSimplifiedBSplineTrajectoryInput(bool& up);
	void initialiseFilters(Eigen::Vector3d initial_value);
	
	//Bspline generation
	void startNewSplineTrajectory();
	std::vector<Eigen::Vector3d> generateBSpline();	
	std::vector<Eigen::Vector3d> generateSimplifiedBSpline();
	
	
    private:
	std::vector<Eigen::Vector3d> control_points;		//stores the input via points for the spline in the environment
	std::vector<Eigen::Vector3d> spline_trajectory;		//stores the final spline trajectory
	std::vector<Eigen::Vector3d> via_points;			//stores the new via points received
	std::vector<FIRFilter> filter;
	int iteration_count;
	int input_point;
	int filter_order;
    };


}

#endif /* COMAUSSGENERATOR_H */

