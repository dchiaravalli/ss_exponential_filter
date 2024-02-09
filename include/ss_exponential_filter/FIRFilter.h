/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Filter.h
 * Author: Davide Chiaravalli, Federico Califano
 *
 * Created on Sep 8, 2016, 2:05 PM
 */

#ifndef FIRFILTER_H
#define FIRFILTER_H

#include <vector>
#include <Eigen/Dense>


namespace ss_exponential_filter{

    class FIRFilter {
    public:
	FIRFilter(double period=1,double Ts=0.001);			//Class Constructor #1
    virtual ~FIRFilter();			//Class Destructor


        //Methods



	//FIR filter implementation

	void initialiseFilter(Eigen::Vector3d initial_value);
    void setFilterParameters(double period, double Ts);
    void addNewInput(Eigen::Vector3d input);
    Eigen::Vector3d evaluateFilterOutput();
    void resetInputPosition();
	int getFilterCellNumber();
	
	
    private:
	float period;							//stores the time period of the spline
	float sampling_time;						//stores the sampling time of the filters used to evaluate the spline
	int cell_number;						//stores the numbers of state memory cells of the filters
    int next_input_position;                //next cell of the filter receiveing the input
	std::vector<Eigen::Vector3d> state;	            //stores the filters states
	

    };


}

#endif /* FIRFILTER_H */

