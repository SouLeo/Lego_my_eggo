#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def main():
	"""
	Initiates a rosnode to communicate when the subject has placed a block and
	pressed a button on the computer keyboard.
    """
    # Set up ROS functionality
    	rospy.init_node('block_ready')

    # Set up the publisher
    	pub = rospy.Publisher('is_block_placed', Bool, queue_size=0)

   	while not rospy.is_shutdown():
   		rospy.sleep(10.0)
   		print('---------------------------------------------------------------')
   		text = raw_input('Please press enter when you have placed the block: ')
		pub.publish(True)

if __name__ == '__main__':
    main()