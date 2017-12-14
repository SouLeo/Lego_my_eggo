#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import sia5_hri_fsm.srv
from tf import TransformListener as TF


def main():
    """
		Testing to see if I can call the handover request with a pose
    """
    lego_pose = PoseStamped()
    color = raw_input('Enter the color block you want to pick up:')

    if color == 'r':
        lego_pose.pose.position.x = 0.745 - 0.460 # 0.489
        lego_pose.pose.position.y = 0.489 - 0.425 # 0.460
        print('Red selected')
    elif color == 'y':
        lego_pose.pose.position.x = 0.745 - 0.295 # 0.339
        lego_pose.pose.position.y = 0.339 - 0.425 # 0.295
        print('Yellow selected')
    elif color == 'g':
        lego_pose.pose.position.x = 0.745 - 0.460 # 0.339
        lego_pose.pose.position.y = 0.339 - 0.425 # 0.460
        print('Green selected')
    elif color == 'b':
        lego_pose.pose.position.x = 0.745 - 0.295 # 0.489
        lego_pose.pose.position.y = 0.489 - 0.425 # 0.295
        print('Blue selected')
    else:
        rospy.logerr('Unknown Color Type Selected')
#    lego_pose.pose.position.x = 0.489
#    lego_pose.pose.position.y = 0.460
    lego_pose.pose.position.z = 0.20-0.088
    lego_pose.pose.orientation.w = 0.0
    lego_pose.pose.orientation.x = 0.493
    lego_pose.pose.orientation.y = -0.870
    lego_pose.pose.orientation.z = 0.0
    lego_pose.header.frame_id = 'base_link'
    lego_pose.header.stamp = rospy.Time.now()

    #TF.waitForTransform('world', 'base_link', rospy.Time(), rospy.Duration(5.0))
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