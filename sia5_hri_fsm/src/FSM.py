#!/usr/bin/env python

# Maintainer: Selma Wanna, slwanna@utexas.edu

import roslib
import rospy
import smach
import smach_ros
from Puzzle import createPatterns, comparePattern, nextBlock
from std_msgs.msg import Bool, Int32

# TODO: Modularize FSM functionality.
# TODO: Replace services and topics into service and topic states in SMACH
# TODO: Do variable reset for variables that switch states

TIMEOUT_SECS = 30
possiblePatterns = []

class Observe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['examine_puzzle','give_block'], input_keys=['is_block_placed_in', 'is_pattern_known'], output_keys=['is_block_placed_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing Observation State')
        global TIMEOUT_SECS
        timeout = rospy.Time.now() + rospy.Duration(TIMEOUT_SECS)
        while not userdata.is_block_placed_in and rospy.Time.now() <= timeout:
            pass    
        if not userdata.is_block_placed_in:
            rospy.loginfo('Timeout Occurred') 
            userdata.is_block_placed_out = False
            return 'give_block'
        rospy.loginfo('Block in place') 
        userdata.is_block_placed_out = True
        if userdata.is_pattern_known:
            return 'give_block'
        else: 
            return 'examine_puzzle'

class ExaminePuzzle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['give_next_block'], input_keys=['is_blocked_placed_out'], output_keys=['is_pattern_known', 'next_block'])

    def execute(self, userdata):
        rospy.loginfo('Executing Examine Puzzle State')
        # TODO: rosservice call that reports block sequence into a string and
        # report into currentPattern
        currentPattern = 'rr'   # example pattern. replace with a rosservice that determines what blocks are shown
        global possiblePatterns
        possiblePatterns = comparePattern(currentPattern, possiblePatterns)
        numSolns = len(possiblePatterns)
        if nextBlock(possiblePatterns, currentPattern) != '0':
            userdata.next_block = nextBlock(possiblePatterns, currentPattern) # Write the next likely block
            # TODO: check to see if user has a block in his/her workspace. i.e.
            # do we need to go to give block?
        if (numSolns == 1):
            rospy.loginfo('Pattern Known!! :D')
            userdata.is_pattern_known = True
        else:
            rospy.loginfo('Pattern still unknown.' + str(numSolns) + 'possibilities')
            userdata.is_pattern_known = False
        return 'give_next_block'

class GiveBlock(smach.State):
    # TODO: this state involves placing a block in the handover zone, the robot
    def __init__(self):
        smach.State.__init__(self, outcomes=['observe'], input_keys=['is_pattern_known', 'next_block'], output_keys=['is_block_placed_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing Give Block State')
        # TODO: rosservice call to place a the next block in the drop off zone
        return 'observe'

def is_block_placed_cb(data):
    rospy.loginfo('block has been placed!')
    sm.userdata.sm_is_block_placed = data.data 

def main():
    # Set up ROS functionality
    rospy.init_node('sia5_fsm')
    # TODO: ROS publisher and subscribers
    rospy.Subscriber('is_block_placed', Bool, is_block_placed_cb)

    # State Machine Setup
    global sm
    sm = smach.StateMachine(outcomes = ['Selma'])
    rospy.loginfo('SM defined')

    # Initialize state machine variables
    sm.userdata.sm_is_block_placed = False
    sm.userdata.sm_is_pattern_known = False
    sm.userdata.sm_next_block = ''
    
    global possiblePatterns 
    possiblePatterns = createPatterns()
   
    with sm:
        # Add states to container
        smach.StateMachine.add('OBSERVE', Observe(), transitions={'give_block':'GIVEBLOCK','examine_puzzle':'EXAMINEPUZZLE'}, remapping={'is_block_placed_in':'sm_is_block_placed', 'is_block_placed_out':'sm_is_block_placed', 'is_pattern_known':'sm_is_pattern_known'})
        smach.StateMachine.add('EXAMINEPUZZLE', ExaminePuzzle(), transitions={'give_next_block':'GIVEBLOCK'}, remapping={'is_block_placed_out':'sm_is_block_placed','is_pattern_known':'sm_is_pattern_known', 'next_block':'sm_next_block' })
        smach.StateMachine.add('GIVEBLOCK', GiveBlock(), transitions={'observe':'OBSERVE'}, remapping={'is_pattern_known':'sm_is_pattern_known','next_block':'sm_next_block', 'is_block_placed)in':'sm_is_block_placed'})

    sis = smach_ros.IntrospectionServer('sia5_fsm', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
