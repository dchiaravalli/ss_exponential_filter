/* 
 * File:   ComauIK.h
 * Author: Davide Chiaravalli,
 *
 * Created on Jen 17, 2018, 4:05 PM
 */

#ifndef COMAUIK
#define COMAUIK

#include <Eigen/Dense>
#include <vector>
#include <kdl/frames_io.hpp>
#include <sensor_msgs/JointState.h>

namespace ss_exponential_filter {

    class ComauIK
    {
        public:
            ComauIK(double tool_offset_x=0,double tool_offset_y=0,double tool_offset_z=187.9425);			    //Class Constructor 
            virtual ~ComauIK();			                                                                //Class Destructor

        //Methods
        bool ik(KDL::Frame &target_frame,KDL::Vector &acceleration,bool wrist_compensation,std::vector<double> &joints);
        bool ik(KDL::Frame &target_frame,Eigen::Vector3d &acceleration,bool wrist_compensation,std::vector<double> &joints);
        bool ik_neg5(KDL::Frame &target_frame,KDL::Vector &acceleration,bool wrist_compensation,std::vector<double> &joints);
        bool ik_neg5(KDL::Frame &target_frame,Eigen::Vector3d &acceleration,bool wrist_compensation,std::vector<double> &joints);
        std::vector<double> fk(std::vector<double> &v);
        KDL::Frame total_fk(std::vector<double> &v);
        Eigen::Vector3d fk_eigen(std::vector<double> &v);
        void setToolOffsets(Eigen::Vector3d offsets);
        void reset();

        
        private:
        //class variables
        std::vector<double> a_c;
        std::vector<double> alpha_c;
        std::vector<double> d_c;
        std::vector<double> teta_c;
        double dc41,tool_offset,e2,ct,phi,teta_d,u1,u2,check_old1,check_old2;
        std::vector<double> output_angles;
        double tool_offset_x,tool_offset_y,tool_offset_z;

        //class methods
        void initialise();
        void setq1(KDL::Vector &v);
        void evaluateparam(KDL::Vector &v);
        void setq2();
        void setq3(KDL::Vector &v);
        KDL::Frame getFrame(double q,double alpha,double a,double dc);
        void setT03(KDL::Frame &frame);  
        void set456(KDL::Rotation rot);
        void set4neg56(KDL::Rotation rot);
        double addition4(double cond,double check);
        double addition6(double cond,double check);
    };
}

#endif /* COMAUIK */
