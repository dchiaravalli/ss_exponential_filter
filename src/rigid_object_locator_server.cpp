#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <ss_exponential_filter/LocateObjectAction.h>
#include <ss_exponential_filter/Liquid_handler.h>

class LocateObjectAction
{
protected:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<ss_exponential_filter::LocateObjectAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

  std::string action_name_;
  // create messages that are used to published feedback/result
  ss_exponential_filter::LocateObjectFeedback feedback_;
  ss_exponential_filter::LocateObjectResult result_;
  ss_exponential_filter::Liquid_handler *handler;
  

public:

  LocateObjectAction(std::string name) :
    as_(nh, name, boost::bind(&LocateObjectAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    handler = new ss_exponential_filter::Liquid_handler(nh,"/atift_sensor/data",2500,true);
    handler->setEnableFinder();
  }

  ~LocateObjectAction(void)
  {
  }

  void executeCB(const ss_exponential_filter::LocateObjectGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(50);
    bool success = true;

    if (this->handler->state == ss_exponential_filter::Liquid_handler::Sensor_state::PRESENT){
        this->handler->state = ss_exponential_filter::Liquid_handler::Sensor_state::MEASURING;
    }
    while((this->handler->state != ss_exponential_filter::Liquid_handler::Sensor_state::PRESENT)&&(ros::ok())){
        r.sleep();
    }
    Eigen::Vector3d meas = this->handler->getToolRigidObjectReverse();
    std_msgs::Float64MultiArray result;
    result.data.resize(3);
    for (int i=0;i<3;i++){
        result.data[i] = meas[i];
    }

    if(success)
    {
      result_.position = result;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_locator_server");

  LocateObjectAction locator("locate_rigid_objects");
  ros::spin();

  return 0;
}