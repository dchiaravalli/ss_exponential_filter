#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include "ss_exponential_filter/SS_parameter_evaluator.h"
#include "ss_exponential_filter/Tool_posefinder.h"
#include <geometry_msgs/Twist.h>

Eigen::Vector3d forces,torques;

void sensor_callback(const geometry_msgs::Twist &msg){
        forces[0]=msg.linear.x;
        forces[1]=msg.linear.y;
        forces[2]=msg.linear.z;
        torques[0]=msg.angular.x;
        torques[1]=msg.angular.y;
        torques[2]=msg.angular.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "testnode");
    ros::NodeHandle n;
    ss_exponential_filter::SS_parameter_evaluator evaluator(0.94,0.002,2);
    ss_exponential_filter::Tool_posefinder tool(0.0275);
    ros::Subscriber sub=n.subscribe("ciao",1,sensor_callback);

    while(ros::ok()){
        tool.setSensorRead(forces,torques);
        evaluator.setLiquidMass(tool.getFluidMass(5));
        //tool.getToolPose(0.005,0.0335)+evaluator.getFluidHeight();
    }
    return 0;
}