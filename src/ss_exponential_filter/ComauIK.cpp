/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SS_filter.cpp
 * Author: Davide Chiaravalli
 * 
 * Created on Jen 17, 2018, 4:05 PM
 */


#include <stdio.h>
#include <cmath>
#include <iostream>
#include "ss_exponential_filter/ComauIK.h"

namespace ss_exponential_filter  {
	

	
    	ComauIK::ComauIK(double tool_offset_x,double tool_offset_y,double tool_offset_z) {
            this->a_c.resize(6);
            this->alpha_c.resize(6);
            this->d_c.resize(6);
            this->teta_c.resize(6);
            this->dc41 =660;
            this->tool_offset_x = tool_offset_x;
            this->tool_offset_y = tool_offset_y;
            this->tool_offset_z = tool_offset_z;
            this->output_angles.resize(6);
            this->e2=0;
            this->ct=0;
            this->phi=0;
            this->u1=0;
            this->u2=0;
            this->check_old1=0;
            this->check_old2=0;
            initialise();

    	}

    	ComauIK::~ComauIK() {
    	}

        void ComauIK::initialise(){
            this->a_c[0] = 150;
            this->a_c[1] = 590;
            this->a_c[2] = 130;
            this->a_c[3] = 0;
            this->a_c[4] = 0;
            this->a_c[5] = 0;
            this->alpha_c[0]=-M_PI/2;
            this->alpha_c[1]=M_PI;
            this->alpha_c[2]=M_PI/2;
            this->alpha_c[3]=-M_PI/2;
            this->alpha_c[4]=M_PI/2;
            this->alpha_c[5]=0;
            this->d_c[0]=450;
            this->d_c[1]=0;
            this->d_c[2]=0;
            this->d_c[3]=647.07;
            this->d_c[4]=0;
            this->d_c[5]=0;
            this->teta_c[0]=0;
            this->teta_c[1]=-M_PI/2;
            this->teta_c[2]=M_PI/2;
            this->teta_c[3]=0;
            this->teta_c[4]=0;
            this->teta_c[5]=0;
            this->teta_d=atan(647.07/130);
        }

        void ComauIK::setq1(KDL::Vector &v){
            this->output_angles[0]=-atan2(v.data[1],v.data[0]);
        }

        bool ComauIK::ik(KDL::Frame &target_frame,KDL::Vector &acceleration,bool wrist_compensation,std::vector<double> &joints){
            KDL::Rotation R;
            if (wrist_compensation){
                double beta=-atan(sqrt(pow(acceleration.data[0],2)+pow(acceleration.data[1],2))/(9.8+acceleration.data[2]));
                double gamma=-M_PI+atan2(acceleration.data[1],acceleration.data[0]);
                if ((fabs(acceleration.data[0])<0.00000001)&&(fabs(acceleration.data[1])<0.00000001)){
                    //std::cout<<"beta "<<std::endl;
                    R=KDL::Rotation::RotZ(-M_PI)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(M_PI);

                }
                else
                    R=KDL::Rotation::RotZ(gamma)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(-gamma);           
            }
            else
                R=target_frame.M;
            KDL::Frame temp;
            KDL::Vector P = target_frame.p-R*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            setq1(P);
            evaluateparam(P);
            setq2();
            setq3(P);
            setT03(temp);
            set456(temp.M.Inverse()*R);
            for (int i=0;i<6;i++){
                joints[i]=this->output_angles[i];
            }
            //if (wrist_compensation==false) joints[5]=M_PI;
            return true;
        }

        bool ComauIK::ik(KDL::Frame &target_frame,Eigen::Vector3d &acceleration,bool wrist_compensation,std::vector<double> &joints){
            KDL::Rotation R;
            if (wrist_compensation){
                double beta=-atan(sqrt(pow(acceleration[0],2)+pow(acceleration[1],2))/(9.8+acceleration[2]));
                double gamma=-M_PI+atan2(acceleration[1],acceleration[0]);
                if ((fabs(acceleration[0])<0.00000001)&&(fabs(acceleration[1])<0.00000001)){
                    std::cout<<"wrist compensate"<<std::endl;
                    R=KDL::Rotation::RotZ(-M_PI)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(M_PI);

                }
                else
                    R=KDL::Rotation::RotZ(gamma)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(-gamma);           
            }
            else
                R=target_frame.M;
            KDL::Frame temp;
            KDL::Vector P = target_frame.p-R*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            setq1(P);
            evaluateparam(P);
            setq2();
            setq3(P);
            setT03(temp);
            set456(temp.M.Inverse()*R);
            for (int i=0;i<6;i++){
                joints[i]=this->output_angles[i];
            }
            //if (wrist_compensation==false) joints[5]=M_PI;
            return true;
        }

        bool ComauIK::ik_neg5(KDL::Frame &target_frame,KDL::Vector &acceleration,bool wrist_compensation,std::vector<double> &joints){
            KDL::Rotation R,R1;
            if (wrist_compensation){
                double beta=-atan(sqrt(pow(acceleration.data[0],2)+pow(acceleration.data[1],2))/(9.8+acceleration.data[2]));
                double gamma=-M_PI+atan2(acceleration.data[1],acceleration.data[0]);
                if ((fabs(acceleration.data[0])<0.00000001)&&(fabs(acceleration.data[1])<0.00000001)){
                    //std::cout<<"beta "<<std::endl;
                    R=KDL::Rotation::RotZ(-M_PI)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(M_PI);
                }
                else
                    R=KDL::Rotation::RotZ(gamma)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(-gamma);         
            }
            else{
                R=target_frame.M;
            }
            KDL::Frame temp;
            KDL::Vector P = target_frame.p-R*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            //std::cout<<R*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z)<<std::endl;
            setq1(P);
            evaluateparam(P);
            setq2();
            setq3(P);
            setT03(temp);
            R1=R*KDL::Rotation::RotX(M_PI);
            set4neg56(temp.M.Inverse()*KDL::Rotation::RotX(M_PI)*R);
            for (int i=0;i<6;i++){
                joints[i]=this->output_angles[i];
            }
            //if (wrist_compensation==false) joints[5]=M_PI;
            return true;
        }

        bool ComauIK::ik_neg5(KDL::Frame &target_frame,Eigen::Vector3d &acceleration,bool wrist_compensation,std::vector<double> &joints){
            KDL::Rotation R,R1;
            if (wrist_compensation){
                double beta=-atan(sqrt(pow(acceleration[0],2)+pow(acceleration[1],2))/(9.8+acceleration[2]));
                double gamma=-M_PI+atan2(acceleration[1],acceleration[0]);
                if ((fabs(acceleration[0])<0.00000001)&&(fabs(acceleration[1])<0.00000001)){
                    //std::cout<<"wrist compensate"<<std::endl;
                    R=KDL::Rotation::RotZ(-M_PI)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(M_PI);

                }
                else
                    R=KDL::Rotation::RotZ(gamma)*KDL::Rotation::RotY(beta)*KDL::Rotation::RotZ(-gamma);          
            }
            else{
                R=target_frame.M;
            }
            KDL::Frame temp;
            KDL::Vector P = target_frame.p-R*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            setq1(P);
            evaluateparam(P);
            setq2();
            setq3(P);
            setT03(temp);
            R1=R*KDL::Rotation::RotX(M_PI);
            set4neg56(temp.M.Inverse()*R1);
            for (int i=0;i<6;i++){
                joints[i]=this->output_angles[i];
            }
            //if (wrist_compensation==false) joints[5]=M_PI;
            return true;
        }

        void ComauIK::evaluateparam(KDL::Vector &v){
            this->e2=pow(v.data[0]-this->a_c[0]*cos(this->output_angles[0]),2)+pow(v.data[1]+this->a_c[0]*sin(this->output_angles[0]),2)+pow(v.data[2]-450,2);
            this->ct=(pow(this->a_c[1],2)+pow(this->dc41,2)-this->e2)/(2*this->a_c[1]*this->dc41);
            this->phi=atan2(v.data[0]/cos(this->output_angles[0])-this->a_c[0],v.data[2]-450);
            //std::cout<<acos((this->e2+pow(this->a_c[1],2)-pow(this->dc41,2))/(2*sqrt(this->e2)*this->a_c[1]))<<std::endl;
        }

        void ComauIK::setq2(){
            this->output_angles[1]=this->phi-acos((this->e2+pow(this->a_c[1],2)-pow(this->dc41,2))/(2*sqrt(this->e2)*this->a_c[1]));
        }

        void ComauIK::setq3(KDL::Vector &v){
            this->output_angles[2]=-(3.0/2*M_PI-this->teta_d-acos(this->ct));
        }

        KDL::Frame ComauIK::getFrame(double q,double alpha,double a,double dc){
            return KDL::Frame(KDL::Rotation(cos(q),-sin(q)*cos(alpha),sin(q)*sin(alpha),sin(q),cos(q)*cos(alpha),-cos(q)*sin(alpha),0,sin(alpha),cos(alpha)),KDL::Vector(a*cos(q),a*sin(q),dc));
        }


        void ComauIK::setT03(KDL::Frame &frame){    
            frame=getFrame(this->teta_c[0]-this->output_angles[0],this->alpha_c[0],this->a_c[0],this->d_c[0])*getFrame(this->output_angles[1]+this->teta_c[1],this->alpha_c[1],this->a_c[1],this->d_c[1])*getFrame(this->output_angles[2]+this->teta_c[2],this->alpha_c[2],this->a_c[2],this->d_c[2]);
        }

        std::vector<double> ComauIK::fk(std::vector<double> &v){
            std::vector<double> ciao;
            KDL::Frame offset_frame;
            offset_frame.M = KDL::Rotation::Identity ();
            offset_frame.p = KDL::Rotation::RotX(M_PI)*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            KDL::Frame frame=getFrame(this->teta_c[0]-v[0],this->alpha_c[0],this->a_c[0],this->d_c[0]);
            frame=frame*getFrame(this->teta_c[1]+v[1],this->alpha_c[1],this->a_c[1],this->d_c[1]);
            frame=frame*getFrame(this->teta_c[2]+v[2],this->alpha_c[2],this->a_c[2],this->d_c[2]);
            frame=frame*getFrame(this->teta_c[3]-v[3],this->alpha_c[3],this->a_c[3],this->d_c[3]);
            frame=frame*getFrame(this->teta_c[4]-v[4],this->alpha_c[4],this->a_c[4],this->d_c[4]);
            frame=frame*getFrame(this->teta_c[5]-v[5],this->alpha_c[5],this->a_c[5],this->d_c[5]);
            frame=frame*offset_frame;
            ciao.push_back(frame.p.data[0]);
            ciao.push_back(frame.p.data[1]);
            ciao.push_back(frame.p.data[2]);
            return ciao;
        }

        KDL::Frame ComauIK::total_fk(std::vector<double> &v){
            KDL::Frame offset_frame;
            offset_frame.M = KDL::Rotation::RotX(M_PI);
            offset_frame.p = KDL::Rotation::RotX(M_PI)*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            KDL::Frame frame=getFrame(this->teta_c[0]-v[0],this->alpha_c[0],this->a_c[0],this->d_c[0]);
            frame=frame*getFrame(this->teta_c[1]+v[1],this->alpha_c[1],this->a_c[1],this->d_c[1]);
            frame=frame*getFrame(this->teta_c[2]+v[2],this->alpha_c[2],this->a_c[2],this->d_c[2]);
            frame=frame*getFrame(this->teta_c[3]-v[3],this->alpha_c[3],this->a_c[3],this->d_c[3]);
            frame=frame*getFrame(this->teta_c[4]-v[4],this->alpha_c[4],this->a_c[4],this->d_c[4]);
            frame=frame*getFrame(this->teta_c[5]-v[5],this->alpha_c[5],this->a_c[5],this->d_c[5]);
            frame=frame*offset_frame;
            return frame;
        }

        Eigen::Vector3d ComauIK::fk_eigen(std::vector<double> &v){
            Eigen::Vector3d result;
            KDL::Frame offset_frame;
            offset_frame.M = KDL::Rotation::Identity ();
            offset_frame.p = KDL::Rotation::RotX(M_PI)*KDL::Vector(this->tool_offset_x,this->tool_offset_y,this->tool_offset_z);
            KDL::Frame frame=getFrame(this->teta_c[0]-v[0],this->alpha_c[0],this->a_c[0],this->d_c[0]);
            frame=frame*getFrame(this->teta_c[1]+v[1],this->alpha_c[1],this->a_c[1],this->d_c[1]);
            frame=frame*getFrame(this->teta_c[2]+v[2],this->alpha_c[2],this->a_c[2],this->d_c[2]);
            frame=frame*getFrame(this->teta_c[3]-v[3],this->alpha_c[3],this->a_c[3],this->d_c[3]);
            frame=frame*getFrame(this->teta_c[4]-v[4],this->alpha_c[4],this->a_c[4],this->d_c[4]);
            frame=frame*getFrame(this->teta_c[5]-v[5],this->alpha_c[5],this->a_c[5],this->d_c[5]);
            frame=frame*offset_frame;
            result[0]=(frame.p.data[0])/1000;
            result[1]=(frame.p.data[1])/1000;
            result[2]=(frame.p.data[2])/1000;
            return result;
        }

        void ComauIK::set456(KDL::Rotation rot){
  //          std::cout<<rot.data[2]<<" "<<rot.data[5]<<" "<<rot.data[6]<<" "<<rot.data[7]<<std::endl;
            this->output_angles[3]=-(atan2(rot.data[5],rot.data[2])+addition4(rot.data[2],rot.data[5]));
            this->output_angles[4]=-(atan2(sqrt(pow(rot.data[2],2)+pow(rot.data[5],2)),rot.data[8]));
            this->output_angles[5]=-(atan2(rot.data[7],-rot.data[6])+addition6(-rot.data[6],rot.data[7]));
            if (this->output_angles[5]<0) this->output_angles[5]=this->output_angles[5]+2*M_PI;
        }

        void ComauIK::set4neg56(KDL::Rotation rot){
            this->output_angles[3]=-(atan2(-rot.data[5],-rot.data[2]));
            this->output_angles[4]=-(atan2(-sqrt(pow(rot.data[2],2)+pow(rot.data[5],2)),rot.data[8]));
            this->output_angles[5]=-(atan2(-rot.data[7],rot.data[6]));
        }

        double ComauIK::addition4(double cond,double check){
            if (cond<0.2){
                if ((this->check_old1<0)&&(check>=0)){
                    this->u1=this->u1-2*M_PI;
                }
                else if ((this->check_old1>0)&&(check<=0)){
                    this->u1=this->u1+2*M_PI;
                }
            }
            this->check_old1=check;
            return this->u1;
        }

        double ComauIK::addition6(double cond,double check){
            if (cond<0){
                if ((this->check_old2<0)&&(check>=0)){
                    this->u2=this->u2-2*M_PI;
                }
                else if ((this->check_old2>0)&&(check<=0)){
                    this->u2=this->u2+2*M_PI;
                }
            }
            this->check_old2=check;
            return this->u2;
        }

        void ComauIK::reset(){
            this->u1=0;
            this->u2=0;
            this->check_old2=0;
            this->check_old1=0;
        }

        void ComauIK::setToolOffsets(Eigen::Vector3d offsets){
            this->tool_offset_x=offsets[0];
            this->tool_offset_y=offsets[1];
            this->tool_offset_z=offsets[2];
        }



}




