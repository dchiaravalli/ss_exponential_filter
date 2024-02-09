#include <ros/ros.h>
#include <ss_exponential_filter/Liquid_handler.h>



int main(int argc, char **argv)
{
    //ros structure initialisation
	ros::init(argc, argv, "object_force_locator");
    ros::NodeHandle n;

    ss_exponential_filter::Liquid_handler handler(n,"/atift_sensor/data",2500,true);
    handler.setEnableFinder();

    ros::Rate r(500);
    while (ros::ok()){
        //std::cout<<"guarda come spinno"<<std::endl;
        ros::spinOnce();
        r.sleep();
    }
    
}

