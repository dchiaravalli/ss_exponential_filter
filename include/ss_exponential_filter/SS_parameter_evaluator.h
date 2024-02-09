/* 
 * File:   SS_parameter_evaluator.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef SS_PARAMETER_EVALUATOR
#define SS_PARAMETER_EVALUATOR

#include<Eigen/Dense>
#include<vector>
#include<trajectory_msgs/JointTrajectory.h>

namespace ss_exponential_filter {

    class SS_parameter_evaluator
    {
        public:
            SS_parameter_evaluator(double sampling_time, double cylinder_radius=1, double fluid_mass=1);			    //Class Constructor 
            virtual ~SS_parameter_evaluator();      //Class Destructor

        //Methods
        void setLiquidMass(double mass);
        void setCylinderRadius(double radius);
        void setSamplingTime(double sampling_time);
        void setSystemParameters(double sampling_time, double radius, double mass);
        std::vector<double> getFilterParameters();
        std::vector<double> getFilterParameters(double mass);
        std::vector<double> getLiquidParameters();
        double getFluidHeight();

        private:

        //class variables
        double cylinder_radius;                             //radius of the cylinder used to carry the liquid
        double fluid_mass;                                  //mass of the liquid to be moved
        double fluid_specific_gravity;                     //fluid specific gravity
        double fluid_volume;                                //volume of the liquid to be moved
        double kinematic_viscosity;                         //kinematic viscosity of the liquid
        double earth_gravity;                               //gravity constant g
        double fluid_height;                                //height of the liquid in the cylinder
        double bessel_ki;                                   //strange unknown parameter
        double pendulum_frequency;                          //frequency of the first mode of the pendulum equivalent to the liquid
        double L1;                                          //strange unknown parameter
        double rei;                                         //strange unknown parameter
        double delta;                                       //delta value of the first mode of the pendulum equivalent to the liquid
        double d1;                                          //strange unknown parameter
        double sampling_time;

        //class methods
        void evaluateFluidParameters();
    };
}

#endif /* SS_PARAMETER_EVALUATOR */
