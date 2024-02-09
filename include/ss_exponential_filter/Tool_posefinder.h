/* 
 * File:   SS_parameter_evaluator.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef TOOL_POSEFINDER
#define TOOL_POSEFINDER

#include<Eigen/Dense>
#include<vector>
#include<trajectory_msgs/JointTrajectory.h>

namespace ss_exponential_filter {

    class Tool_posefinder
    {
        public:
            Tool_posefinder(double sensor_offset);			    //Class Constructor 
            virtual ~Tool_posefinder();      //Class Destructor

        //Methods
        void setSensorForceRead(Eigen::Vector3d forces);
        void setSensorTorqueRead(Eigen::Vector3d torques);
        void setSensorOffset(double sensor_offset);
        void setSensorRead(Eigen::Vector3d forces, Eigen::Vector3d torques);
        double getFluidMass(double offset);
        Eigen::Vector3d getToolPose(double bessel_thickness=0,double sensor_to_joint=0);

        private:

        //class variables
        Eigen::Vector3d forces,torques,z_force;
        double sensor_offset;

        //class methods

    };
}

#endif /* TOOL_POSEFINDER */
