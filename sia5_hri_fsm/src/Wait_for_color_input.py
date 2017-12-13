#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def main():
  """
    Initiates a rosnode to communicate when the subject has placed a block and
    pressed a button on the computer keyboard.
  """
  
  # Initialize the output string of block colors
  output_str = ''

  # Set up ROS functionality
  rospy.init_node('block_ready')

  # Set up the publisher
  pub = rospy.Publisher('is_block_placed', Bool, queue_size=0)
  
  while not rospy.is_shutdown():
    rospy.sleep(10.0)
    print('---------------------------------------------------------------')
    print('Please enter the first letter(s) (lower-case) of the color block(s) you placed in order and then press enter:')
    print('Red = r, Yellow = y, Green = g, Blue = b')
    print('Example: One red block = r, A blue and yellow = by')
    text = raw_input()
    output_str = output_str + text
    # Ready to output this string to the service or topic
    pub.publish(True)

if __name__ == '__main__':
    main()