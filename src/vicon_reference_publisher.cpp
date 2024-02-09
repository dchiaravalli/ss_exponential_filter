#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <ss_exponential_filter/FIRFilter.h>

int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "vicon_reference_publisher");
    ros::NodeHandle n;

	tf::TransformListener listener;														//listener to receive values from vicon
	tf::StampedTransform transform;	
    bool first = true;
    Eigen::Vector3d result,new_value;
    int interp=0;
    geometry_msgs::Pose pose;
    ss_exponential_filter::FIRFilter filter(1,0.2);
    ros::Publisher pub = n.advertise<geometry_msgs::Pose>("vicon_interpolation",1);

    ros::Rate r(500);
    while(ros::ok())
	{
        if (first){
            try
            {
                listener.lookupTransform( "vicon", "pen2", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                //ROS_ERROR("%s",ex.what());
            }
            new_value[0]=transform.getOrigin().x();
            new_value[1]=transform.getOrigin().y();
            new_value[2]=transform.getOrigin().z();
            filter.initialiseFilter(new_value);
            filter.addNewInput(new_value);
            first = false;
            continue;
        }
        if (interp<4){
            filter.addNewInput(new_value);
            interp++;
        }
        else{
            try
            {
                listener.lookupTransform( "vicon", "pen2", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                //ROS_ERROR("%s",ex.what());
            }
            new_value[0]=transform.getOrigin().x();
            new_value[1]=transform.getOrigin().y();
            new_value[2]=transform.getOrigin().z();          
            interp = 0; 
            filter.addNewInput(new_value);
        }
        result = filter.evaluateFilterOutput();
        pose.position.x = result[0];
        pose.position.y = result[1];
        pose.position.z = result[2];
        pub.publish(pose);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}

