#include <ros/ros.h>
#include "move_down.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo");
  ros::AsyncSpinner spinner(5);
  spinner.start();
  ros::NodeHandle node_;

  // ROS_INFO_STREAM("Starting multiuse_workcell demo");
  // MoveDown moveDownTest(node_);
  // moveDownTest.moveDown();
  // ROS_INFO_STREAM("multiuse_workcell demo complete.");
  // return 0;


  MoveDown moveDownTest(node_);
  ros::ServiceServer ss = node_.advertiseService("handover", &MoveDown::handover, &moveDownTest);


  // ros::spin();

  return 0;
}