#include <ros/ros.h>
#include <iostream>
#include "sia5_hri_fsm/move_down.h"

int main(int argc, char **argv)
{
  // Initialize the ros move_interface_node
  ros::init(argc, argv, "demo_node");

  // Instantiate mover class
  ros::NodeHandle n; 
  ros::AsyncSpinner spinner(3);
  spinner.start();
  {
    ros::ServiceServer service = n.advertiseService("handover", handover);
    MoveDown moveTest(n);
    moveTest.moveDown();
  }
  std::cout << "hi" << std::endl;
  return 0;
}

