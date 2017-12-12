#include "move_down.h"

#define BUFFER_SIZE 64

MoveDown::MoveDown(ros::NodeHandle nh) :
  n(nh),
  cc()
{
  markerPub = n.advertise<visualization_msgs::Marker>("/marker",1);
}

bool MoveDown::handover(sia5_hri_fsm::Handover::Request&  req,
         sia5_hri_fsm::Handover::Response& res)
{
  moveDown();
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
  bowlPos.resize(7);
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
    ros::Duration(3.0).sleep();
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
  bool handover_bool = false;
  // Set moves to half speed 
  mi->setVelocityScaling(0.3);
  gi->setSpeed(0);
  gi->setForce(40);
  activateGripper();
  gi->setMode(RSGripperInterface::MODE_PINCH);
  openGripper();


  double x = 0.4;
  double y = 0.35;
  double z = 0.22;
  double ww = 0.0;
  double xx = 0.964;
  double yy = -0.267;
  double zz = 0.0;

  // moveToPose(x,
  //            y,
  //            z,
  //            xx,
  //            yy,
  //            zz);
  // ros::Duration(0.3).sleep();

  geometry_msgs::Pose ref = createPose(x, y, z,
    ww, xx, yy, zz);

  geometry_msgs::PoseStamped posey_pose;

  posey_pose.pose = ref;

  mi->moveCart(posey_pose, 1.0, false);

  // moveToPose(x,
  //          y,
  //          z-0.1,
  //          xx,
  //          yy,
  //          zz);
  closeGripper();

  if(!moveWithInput(bowlPos, "bowl", false))
    return;

  ros::Duration(0.3).sleep();
  
  openGripper();

  handover_bool = true;
  

  
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

void MoveDown::openGripper()
{
  gi->setPosition(80); //107 is fully closed for pinch mode
}

void MoveDown::activateGripper()
{
gi->activate();
}

void MoveDown::closeGripper(bool slow)
{
  if(!slow)
  {
    gi->setPosition(113);
  }
  else
  {
    for(int i = 80; i<=113; i++)
    {
      gi->setPosition(i);
      ros::Duration(0.05).sleep();
    }
  }    
}

bool MoveDown::moveWithInput(std::vector<double> joints, std::string name, bool pause)
{
  char userInput = 'r';
  while(userInput == 'r' || userInput == 'R')
  {
    // Move to home position
    mi->moveJoints(joints, 1.0);
    MoveInterface::MoveStat moveStat = mi->waitForStatus();
    
    // Exit if move returns error
    if(MoveInterface::STATUS_WAITING != moveStat)
    {
      userInput = 'x';
      ROS_ERROR_STREAM("There was an error in the joint move to " << name << ". Enter 'r' to retry, 'c' to continue anyway or 'q' to quit.");
      std::cin >> userInput;
      flushInput();
      while(userInput != 'c' && userInput != 'C' && userInput != 'r' && userInput != 'R' && userInput != 'q' && userInput != 'Q')
      {
        ROS_ERROR_STREAM(userInput << " is not a valid response. Please enter valid input");
        std::cin >> userInput;
        flushInput();
      }
    }
    else
    {
      if(pause)
      {
        ROS_INFO_STREAM("Move to " << name << " position complete. Press enter to continue.");
        std::cin.get();
      }
      else
      {
        ROS_INFO_STREAM("Move to " << name << " position complete.");
      }
      userInput = 'c';
    }
  }
  // Quit the program if user asks us to
  if(userInput == 'q' || userInput == 'Q')
    return false;
  
  return true;
}

void MoveDown::flushInput()
{
  char ch;
  while ( std::cin.get ( ch ) && ch != '\n' )
    ;
}

geometry_msgs::Pose MoveDown::createPose(double x, double y, double z,
    double ww, double xx, double yy, double zz)
{
  geometry_msgs::Pose ret;
  ret.position.x = x;
  ret.position.y = y;
  ret.position.z = z;
  ret.orientation.w = ww;
  ret.orientation.x = xx;
  ret.orientation.y = yy;
  ret.orientation.z = zz;
  return ret;
}

void MoveDown::showArrow(geometry_msgs::PoseStamped tempPose) {
  visualization_msgs::Marker pointer;
  pointer.pose = tempPose.pose;
  tempPose.header.stamp = ros::Time::now();
  pointer.scale.x=0.05;
  pointer.scale.y=0.01;
  pointer.scale.z=0.01;
  pointer.color.a=1.0;
  pointer.color.r=1.0;
  pointer.color.g=1.0;
  pointer.color.b=1.0;
  pointer.header.frame_id = std::string(tempPose.header.frame_id);
  pointer.type=visualization_msgs::Marker::ARROW;
  markerPub.publish(pointer);

  // publishPoseAsTransform(tempPose, "grasp_pose");
}