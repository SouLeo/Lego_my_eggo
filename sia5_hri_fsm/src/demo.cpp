#include <ros/ros.h>
#include "move_down.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "something_else");
  ros::NodeHandle node_;
  //ros::AsyncSpinner spinner(5);
  //spinner.start();

  // ROS_INFO_STREAM("Starting multiuse_workcell demo");
  // MoveDown moveDownTest(node_);
  // moveDownTest.moveDown();
  // ROS_INFO_STREAM("multiuse_workcell demo complete.");
  // return 0;


  MoveDown moveDownTest(node_);
  moveDownTest.initialize();
  ros::ServiceServer ss = node_.advertiseService("sia5_hri_fsm/Handover", &MoveDown::handover, &moveDownTest);
  ROS_INFO_STREAM("handover service initiated");  
  // moveDownTest.run();


  ros::spin();

  return 0;
}