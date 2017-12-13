#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped

print "============ Starting"
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)           

rospy.sleep(5.0)

transform2 = tf_buffer.lookup_transform("world",
	 								   "eef",
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
print "======== final pose: %s" % transform2