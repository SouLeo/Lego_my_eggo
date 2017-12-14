#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sia5_hri_fsm.srv


def main():
    """
		Testing to see if I can call the handover request with a pose
    """
    print('waiting for service')
    rospy.wait_for_service('sia5_hri_fsm/Pattern')
    print('service found')
    pattern_call = rospy.ServiceProxy('sia5_hri_fsm/Pattern', sia5_hri_fsm.srv.Pattern)
    resp1 = pattern_call()
    print('service called')
    print(resp1)

if __name__ == '__main__':
	rospy.init_node('test_pose_call')
	main()