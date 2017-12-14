#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sia5_hri_fsm.srv


def main():
    """
		Testing to see if I can call the handover request with a pose
    """
    lego_pose = PoseStamped()
    lego_pose.header.frame_id = 'world'
    lego_pose.header.stamp = rospy.Time.now()
    lego_pose.pose.position.x = 0.489
    lego_pose.pose.position.y = 0.460
    lego_pose.pose.position.z = 0.17
    lego_pose.pose.orientation.w = 0.0
    lego_pose.pose.orientation.x = 0.964
    lego_pose.pose.orientation.y = -0.267
    lego_pose.pose.orientation.z = 0.0

    print('waiting for service')
    rospy.wait_for_service('sia5_hri_fsm/Handover')
    print('service found')
    handover_call = rospy.ServiceProxy('sia5_hri_fsm/Handover', sia5_hri_fsm.srv.Handover)
    handover_bool = handover_call(lego_pose)
    print('service called')
    print(handover_bool)

if __name__ == '__main__':
	rospy.init_node('test_pose_call')
	main()