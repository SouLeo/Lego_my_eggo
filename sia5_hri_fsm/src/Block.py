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
    block_pose.header.frame_id = "/block_pose"
    block_pose.header.stamp = rospy.Time.now()
    if color == 'r':
        block_pose.pose.position.x = 0
        block_pose.pose.position.y = 0
    elif color == 'y':
        block_pose.pose.position.x = 1
        block_pose.pose.position.y = 1
    elif color == 'g':
        block_pose.pose.position.x = 2
        block_pose.pose.position.y = 2
    elif color == 'b':
        block_pose.pose.position.x = 3
        block_pose.pose.position.y = 3
    else:
        rospy.logerr('Unknown Color Type Selected')
    return  block_pose
