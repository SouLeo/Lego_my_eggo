#include "sia5_hri_fsm/move_down.h"

#define BUFFER_SIZE 64

MoveDown::MoveDown(ros::NodeHandle nh) :
  n(nh),
  cc()
{
}

bool MoveDown::handover(sia5_hri_fsm::MoveDown::Request  &req,
         sia5_hri_fsm::MoveDown::Response &res)
{
  moveDown();
  ROS_INFO("request: ", (geometry_messages::PoseStamped)tempPose);
  ROS_INFO("sending back response: ", (bool)handover_bool);
  return true;
}

MoveDown::~MoveDown()
{
    delete oi;
    delete gi;
}

bool MoveDown::initialize()
{
  // Get config data
  std::string moveGroup, fixedFrame, velFrame, ftFrame, controlFrame, ftAddress, velTopic;
  if(n.getParam("/config_data/move_group", moveGroup) &&
     n.getParam("/config_data/fixed_frame", fixedFrame) &&
     n.getParam("/config_data/vel_frame", velFrame) &&
     n.getParam("/config_data/ft_frame", ftFrame) &&
     n.getParam("/config_data/control_frame", controlFrame) &&
     n.getParam("/config_data/vel_topic", velTopic) &&
     n.getParam("/config_data/ft_address", ftAddress) &&
     n.getParam("/config_data/bowl_pos", bowlPos))
  {
    // Initialize contact_control
    cc.setFTAddress(ftAddress);
    cc.setVelTopic(velTopic);
    cc.initialize(moveGroup,fixedFrame,velFrame,ftFrame,controlFrame);
    
    mi = cc.getMI();
    
    oi = new CollisionInterface(n);
    gi = new RSGripperInterface();
    ros::Duration(1.0).sleep();
    return true;
  }
  else
  {
    ROS_ERROR("Unable to get config data from param server. Ending demo.");
    return false;
  }
  
}

void MoveDown::run()
{
  // Tell FT driver to start getting data
  activateGripper();
  // Set moves to half speed 
  mi->setVelocityScaling(0.3);
  gi->setSpeed(0);
  gi->setForce(40);
  gi->setMode(RSGripperInterface::MODE_PINCH);
  openGripper();
  double x = 0.2;
  double y = 0.2;
  double z = 0.2;
  moveToPose(x,
             y,
             z+0.1,
             0,
             0,
             0);
  ros::Duration(0.3).sleep();

  moveToPose(x,
           y,
           z,
           0,
           0,
           0);
  closeGripper();

  if(!moveWithInput(bowlPos, "bowl", false))
    return;

  ros::Duration(0.3).sleep();
  
  // Bias the netft sensor
  

  
  ROS_INFO_STREAM("Complete.");
}

void MoveDown::moveDown()
{
       // Initialize mover
   if(initialize())
   {
     // Run the test
     run();
   }
  ROS_INFO_STREAM("Bye.");
}

void MoveDown::moveToPose(float x, float y, float z,
    float xr, float yr, float zr) {
  geometry_msgs::PoseStamped tempPose;
  tempPose.header.frame_id = "world";

  tempPose.pose.position.x = x;
  tempPose.pose.position.y = y;
  tempPose.pose.position.z = z;

  tf::Quaternion orientation;
  orientation.setEuler(yr, xr, zr);
  tf::quaternionTFToMsg(orientation,tempPose.pose.orientation);

  showArrow(tempPose);

  mi->moveArm(tempPose, 1.0, false);
  if(mi->waitForStatus() == MoveInterface::STATUS_ERROR)
    throw std::exception();
}