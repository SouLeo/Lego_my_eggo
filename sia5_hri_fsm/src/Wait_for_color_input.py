#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import sia5_hri_fsm.srv


class HRI():

  # Initialize the output string of block colors
  pattern_str = ''
  output_str = ''

  def __init__(self):
    """
      Initiates a rosnode to communicate when the subject has placed a block and
      pressed a button on the computer keyboard.
    """

    # Set up ROS functionality
    rospy.init_node('block_ready')

    # Set up the publisher
    pub = rospy.Publisher('is_block_placed', Bool, queue_size=0)
    serv = rospy.Service('sia5_hri_fsm/Pattern', sia5_hri_fsm.srv.Pattern, self.current_pattern)

    while not rospy.is_shutdown():
      #rospy.sleep(10.0)
      print('---------------------------------------------------------------')
      print('Please enter the first letter(s) (lower-case) of the color block(s) you placed in order and then press enter:')
      print('Red = r, Yellow = y, Green = g, Blue = b')
      print('Example: One red block = r, A blue and yellow = by')
      text = raw_input()
      self.output_str = self.pattern_str + text
      # Ready to output this string to the service or topic
      pub.publish(True)

  def current_pattern(self, req):
    print('service called')
    print(self.output_str)
    return self.output_str

if __name__ == '__main__':
    HRI = HRI()