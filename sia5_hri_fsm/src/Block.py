#!/usr/bin/env python
"""
    This code translates block color to block
    position. It can be converted into a class
    if needed, but given our current needs, remains
    as a function file.

    Maintainer: Selma Wanna slwanna@utexas.edu
"""

import rospy
from geometry_msgs.msg import PoseStamped

#class Block(object):
#    def __init__(self, color):
#        self.color = color
#        self.pose = PoseStamped()
#        if color == 'r':
#            self.pose.pose.x =
#            self.pose.pose.y =
#        else if color == 'y':
#            self.pose.pose.x =
#            self.pose.pose.y =
#        else if color == 'g':
#            self.pose.pose.x =
#            self.pose.pose.y =
#        else if color == 'b':
#            self.pose.pose.x =
#            self.pose.pose.y =
#        else:
#            rospy.logerr('Unknown Color Type Selected')
def get_pose(color):
    """ Function: getPose
        Description: Returns pose location of blocks
                     of a certain color
        Input: color (string)
        Output: pose (ROS PoseStamped)
    """
    block_pose = PoseStamped()
    block_pose.header.frame_id = "base_link"
    block_pose.header.stamp = rospy.Time.now()
    block_pose.pose.position.z = 0.20-0.088
    block_pose.pose.orientation.w = 0.0
    block_pose.pose.orientation.x = 0.493
    block_pose.pose.orientation.y = -0.870
    block_pose.pose.orientation.z = 0.0
    if color == 'r':
        block_pose.pose.position.x = 0.745 - 0.460 # 0.489
        block_pose.pose.position.y = 0.489 - 0.425 # 0.460
    elif color == 'y':
        block_pose.pose.position.x = 0.745 - 0.295 # 0.339
        block_pose.pose.position.y = 0.339 - 0.425 # 0.295
    elif color == 'g':
        block_pose.pose.position.x = 0.745 - 0.460 # 0.339
        block_pose.pose.position.y = 0.339 - 0.425 # 0.460
    elif color == 'b':
        block_pose.pose.position.x = 0.745 - 0.295 # 0.489
        block_pose.pose.position.y = 0.489 - 0.425 # 0.295
    else:
        rospy.logerr('Unknown Color Type Selected')
    return  block_pose
