#include <ros/ros.h>
#include <contact_control.h>
#include <string>
#include <vector>
#include <math.h>
#include <future>
#include <collision_interface/collision_interface.h>
#include <grasp_interface/rs_gripper_interface.h>
#include <netft_utils_lean.h>
#include "stdio.h"
#include "linux/input.h"
#include <sys/stat.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <iostream>

class MoveDown
{
public:
  MoveDown(ros::NodeHandle nh);
  ~MoveDown();
  // Action method evaluates commands that have been received and calls appropriate method
  bool initialize();
  void run();
  void cleanup();
  void moveDown();
  
private:
  // Node handle
  ros::NodeHandle n;                                // ROS node handle
  ros::Subscriber sub_;
  // Move interface
  ContactControl cc;                                // Interface for contact control tasks
  MoveInterface* mi;                                // Interface for Moveit moves
  CollisionInterface* oi;                           // Interface to add objects to planning scene
  RSGripperInterface* gi;                           // Gripper interface
  
  // Gripper functions
  void openGripper();
  void closeGripper(bool slow = false);
  void moveToPose(float x, float y, float z,
  float xr, float yr, float zr);
  // ROS publisher
  ros::Publisher gripperPub;
};

