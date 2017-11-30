#!/usr/bin/env python

# Maintainer: Selma Wanna, slwanna@utexas.edu

import roslib
import rospy
import smach
import smach_ros
from Puzzle import createPatterns, comparePattern, nextBlock
from std_msgs.msg import Bool, Int32

# TODO: Replace services and topics into service and topic states in SMACH
# TODO: Reformat to 80 tw when I don't hate life

TIMEOUT_SECS = 30
NUMERRGUESS = 0
possiblePatterns = []

class Observe(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['examine_puzzle'], input_keys=['is_block_placed_in'], output_keys=[''])

    def execute(self, userdata):
        rospy.loginfo('Executing Observation State')
        global TIMEOUT_SECS
        timeout = rospy.Time.now() + rospy.Duration(TIMEOUT_SECS)
        while not userdata.is_block_placed_in and rospy.Time.now() <= timeout:
            pass    
        if not userdata.is_block_placed_in:
            rospy.loginfo('Timeout Occurred') 
        else:
            rospy.loginfo('Block in place') 
        return 'examine_puzzle'

class ExaminePuzzle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['give_next_block'], input_keys=['is_blocked_placed_out', 'is_pattern_known'], output_keys=['next_block'])

    def execute(self, userdata):
        rospy.loginfo('Executing Examine Puzzle State')
        global possiblePatterns
        global NUMGUESSES
        # TODO: rosservice call that reports block sequence into a string and
        # report into currentPattern
        currentPattern = 'rr'   # example pattern. replace with a rosservice that determines what blocks are shown
        if not userdata.is_pattern_known:
            possiblePatterns = comparePattern(currentPattern, possiblePatterns)
            numSolns = len(possiblePatterns)
            # TODO: check to see if user has a block in his/her workspace. i.e.
            # do we need to go to give block?
            if (numSolns == 1):
                rospy.loginfo('Pattern Discovered!')
                userdata.is_pattern_known = True
                newPiece = nextBlock(possiblePatterns, currentPattern) # Write the next likely block
            else:
                rospy.loginfo('Pattern still unknown. ' + str(numSolns) + ' possibilities')
                NUMERRGUESS = NUMERRGUESS + 1
                # TODO: make another nextBlock function w/ additional param
                newPiece = nextBlock(possiblePatterns, currentPattern, NUMERRGUESS)
        else:
            newPiece = nextBlock(possiblePatterns, currentPattern) # Write the next likely block
            rospy.loginfo('Pattern is known!')
        userdata.next_block = newPiece
        rospy.loginfo('Giving Block ' + newPiece)
        return 'give_next_block'


class GiveBlock(smach.State):
    # TODO: this state involves placing a block in the handover zone, the robot,
    # this may turn into a service state type or an action state type. Talk w/
    # Christina
    def __init__(self):
        smach.State.__init__(self, outcomes=['observe'], input_keys=['next_block'], output_keys=['is_block_placed_in'])

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
    # ROS publisher and subscribers
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
        smach.StateMachine.add('OBSERVE', Observe(), transitions={'examine_puzzle':'EXAMINEPUZZLE'}, remapping={'is_block_placed_in':'sm_is_block_placed'})
        smach.StateMachine.add('EXAMINEPUZZLE', ExaminePuzzle(), transitions={'give_next_block':'GIVEBLOCK'}, remapping={'is_block_placed_out':'sm_is_block_placed','is_pattern_known':'sm_is_pattern_known', 'next_block':'sm_next_block' })
        smach.StateMachine.add('GIVEBLOCK', GiveBlock(), transitions={'observe':'OBSERVE'}, remapping={'is_pattern_known':'sm_is_pattern_known','next_block':'sm_next_block', 'is_block_placed_in':'sm_is_block_placed'})

    sis = smach_ros.IntrospectionServer('sia5_fsm', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
