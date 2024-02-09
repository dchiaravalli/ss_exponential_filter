/* 
 * File:   SS_filter.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef SS_FILTER_H
#define SS_FILTER_H

#include<Eigen/Dense>
#include<vector>
#include<trajectory_msgs/JointTrajectory.h>

namespace ss_exponential_filter {

    class SS_filter
    {
        public:
            SS_filter(double period=1,double sampling_time=0.001,double deformedImpulseResponseFactor=-0.4);    //Class Constructor 
            virtual ~SS_filter();			                                                                    //Class Destructor

        //Methods

            //class attribute definers
            void setFilterPeriod(double period);                                                                //period of the filter
            void setFilterSamplingTime(double sampling_time);                                                   //sampling time
            void setFilterResponseShaper(double filter_response_shaper);                                        //reshaping factor of impulse response
            void setFilterInitialState(Eigen::Vector3d &initial_state);                                         //filter initial state
            void setFilterParameters(double period, double sampling_time, double filter_response_shaper=-0.4);  //filter parameters all in one
            void setFilterParameters(std::vector<double> parameters);                                           //filter parameters all in one
            void setFilterPositionOffset(Eigen::Vector3d &position_offset);                                     //offset between master and robot trajectories
            void setFilterCurve(int filter_curve);                                                              //curvature samples
            void setSafetyOffset(Eigen::Vector3d safety_offset);                                                //safety offsets
            void resetFilterState();                                                                            //reset values to default

            //class attribute readers
            const double getFilterPeriod();                                                                     //period of the filter
            const double getFilterSamplingTime();                                                               //sampling time 
            const double getFilterResponseShaper();                                                             //reshaping factor of impulse response
            const double getFilterSampleNumber();                                                               //filter sample number
            const Eigen::Vector3d getFilterInitialState();                                                      //filter initial state
            std::vector<double> getFilterParameters();                                                          //filter parameters all in one
            std::vector<Eigen::Vector3d> getFilteredTrajectory();                                               //returns filtered trajectory
            std::vector<Eigen::Vector3d> getFilteredAcceleration();                                             //returns filtered acceleration

            //ros message to eigen converters
            std::vector<Eigen::Vector3d> jointTrajectoryToEigen(trajectory_msgs::JointTrajectory &trajectory);
            Eigen::Vector3d jointTrajectoryPointToEigen(trajectory_msgs::JointTrajectoryPoint &point);
            trajectory_msgs::JointTrajectory EigenToJointTrajectoryPos(std::vector<Eigen::Vector3d> positions);
            trajectory_msgs::JointTrajectory EigenToJointTrajectoryAcc(std::vector<Eigen::Vector3d> accelerations);
            trajectory_msgs::JointTrajectory EigenToJointTrajectory(std::vector<Eigen::Vector3d> positions,std::vector<Eigen::Vector3d> accelerations);
            trajectory_msgs::JointTrajectoryPoint EigenToJointTrajectorPoint(Eigen::Vector3d position,Eigen::Vector3d acceleration);

            //SS filter implementation
            std::vector<Eigen::Vector3d> evaluateOnlineSingleFilterStep(Eigen::Vector3d &input);                        //single step evaluation of the filter for online control (continuously receive a new point each cycle)
            void evaluateFilteredTrajectory(std::vector<Eigen::Vector3d> &trajectory);                                  //evaluation of filtered trajectory given an initial trajectory
            void evaluateFilteredTrajectory(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d &initial_state);   //evaluation of filtered trajectory given an initial trajectory
            void evaluateViaPointTrajectory(std::vector<Eigen::Vector3d> &trajectory,Eigen::Vector3d &initial_state);   //evaluation of filtered trajectory given an initial trajectory passing exactly through each via point
        
        private:
            //class variables
            double period;							                        //stores the time period 
            double sampling_time;						                    //stores the sampling time of the filter
            double filter_response_shaper;                                  //stores the factor reshaping of the impulse response of the filter
            int sample_number;                                              //stores the number of samples per period
            int filter_curve;                                               //stores the number of samples used for joining consecutive trajectories
            Eigen::Vector3d filter_initial_state;                           //stores the initial state for a filter evaluation
            Eigen::Vector3d filter_position_offset;                         //stores the offset reference to produce the trajectory starting at chosen point
            Eigen::Vector3d safety_offset;                                  //stores safety margins from workspace limits (not used)
            std::vector<Eigen::Vector3d> filtered_trajectory;               //stores the filtered trajectory produced by the filter
            std::vector<Eigen::Vector3d> filtered_acceleration;             //stores the acceleration for the trajectory produced by the filter
            std::vector<Eigen::Vector3d> input_vector;	                    //stores the last ($sample_number+2) input received by the filter
            std::vector<Eigen::Vector3d> output_pos_vector;	                //stores the last three output produced by the position filter
            std::vector<Eigen::Vector3d> output_acc_vector;	                //stores the last three output produced by the acceleration filter
            double A,B,C,D,E,F,H,I;                                         //Constants for the exponential filter


            //class methods
            void inputVectorInitialiser();                                  //initialise filter input vector 
            void outputVectorInitialiser();                                 //initialise filter output vector
            void evaluateFilterConstants();                                 //evaluation of filter costants given filter parameters
            void evaluateSingleFilterStep(Eigen::Vector3d &input);          //evaluation of a single filter step
            Eigen::Vector3d evaluateFilterPositionFunction();               //evaluation of filtered position for a single filter step
            Eigen::Vector3d evaluateFilterAccelerationFunction();           //evaluation of filtered acceleration for a single filter step
    };
}

#endif /* SS_FILTER_H */
