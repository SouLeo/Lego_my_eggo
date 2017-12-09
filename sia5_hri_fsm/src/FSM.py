#!/usr/bin/env python
"""
    This code contains the main function to run the SIA-5 finite state
    machine for Dr. Thomaz's HRI Project.

    The state machine is comprised of three states:
    1. Observe
    2. ExaminePuzzle
    3. GiveBlock
    4. TriggerUpdateCheck
    5. TriggerPatternStatus
    6. TriggerHandover

    Maintainer: Selma Wanna, slwanna@utexas.edu
"""
import rospy
import smach
import smach_ros
from smach_ros import ServiceState
from Puzzle import (create_patterns, create_starter_patterns,
                    compare_pattern, next_block, is_piece_available)
from Block import get_pose
from std_msgs.msg import Bool
import sia5_hri_fsm.srv

SM = None
TIMEOUT_SECS = 30
RAND = True
NUMERRGUESS = 0
POSSIBLE_PATTERNS = []
STARTER_PATTERNS = ''

class Observe(smach.State):
    """
        The Observation state checks to see if a user has filled in a
        new block. it will timeout in 30 seconds if no block is placed.
        Regardless of timeout it will enter the state ExaminePuzzle
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['examine_puzzle'],
                             input_keys=['is_block_placed_in'],
                             output_keys=[''])

    def execute(self, userdata):
        rospy.loginfo('Executing Observation State')
        timeout = rospy.Time.now() + rospy.Duration(TIMEOUT_SECS)
        while not userdata.is_block_placed_in and rospy.Time.now() <= timeout:
            pass
        if not userdata.is_block_placed_in:
            rospy.loginfo('Timeout Occurred')
        else:
            rospy.loginfo('Block in place')
        return 'examine_puzzle'

class ExaminePuzzle(smach.State):
    """
        The ExaminePuzzle state decodes the camera image into a string
        and refines the possible patterns the human is constructing.
        It will provide the next block to handover to the human
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['give_next_block'],
                             input_keys=['is_blocked_placed_out', 'is_pattern_known'],
                             output_keys=['next_block'])

    def execute(self, userdata):
        rospy.loginfo('Executing Examine Puzzle State')
        global POSSIBLE_PATTERNS
        global NUMERRGUESS
        # TODO: rosservice call that reports block sequence into a string and
        # report into current_pattern
        current_pattern = 'rr'   # use rosservice that determines what blocks are shown
        if not userdata.is_pattern_known:
            POSSIBLE_PATTERNS = compare_pattern(current_pattern, POSSIBLE_PATTERNS)
            num_solns = len(POSSIBLE_PATTERNS)
            if num_solns == 1:
                rospy.loginfo('Pattern Discovered!')
                userdata.is_pattern_known = True
                new_piece = next_block(RAND, POSSIBLE_PATTERNS, current_pattern) # next likely block
            else:
                rospy.loginfo('Pattern still unknown. ' + str(num_solns) + ' possibilities')
                NUMERRGUESS = NUMERRGUESS + 1
                new_piece = next_block(RAND, POSSIBLE_PATTERNS, current_pattern, NUMERRGUESS)
        else:
            new_piece = next_block(RAND, POSSIBLE_PATTERNS, current_pattern) # next likely block
            rospy.loginfo('Pattern is known!')
        userdata.next_block = new_piece
        rospy.loginfo('Giving Block ' + new_piece)
        return 'give_next_block'


class GiveBlock(smach.State):
    """
        The GiveBlock state provides the needed block (determined by the
        ExaminePuzzle state) to the shared human-robot shared workspace
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['observe', 'trigger_handover'],
                             input_keys=['next_block'],
                             output_keys=['is_block_placed_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing Give Block State')
        global STARTER_PATTERNS
        handover_block = is_piece_available(userdata.next_block, STARTER_PATTERNS)
        if handover_block:
            rospy.loginfo('No action taken')
            STARTER_PATTERNS = STARTER_PATTERNS.replace(userdata.next_block, '')
            # TODO: rosservice: visually check for updated puzzle
            # change the return statement from observe to the service state
            # which will check if puzzle updated
            return 'observe'
        else:
            block_pose = get_pose(userdata.next_block)
            rospy.loginfo('Pose is ' + str(block_pose.pose.position.x) + ', '
                          + str(block_pose.pose.position.y))
            return 'trigger_handover'

def is_block_placed_cb(data):
    """
        Function:    is_block_placed_cb
        Input:       block placed? (boolean)
        Output:      none
    """
    rospy.loginfo('block has been placed!')
    SM.userdata.sm_is_block_placed = data.data

def main():
    """
        Initializes and runs the state machine for the SIA-5.
        State machine is viewable with
        rosrun smach_viewer smach_viewer.py
    """
    # Set up ROS functionality
    rospy.init_node('sia5_fsm')

    # ROS publisher and subscribers
    rospy.Subscriber('is_block_placed', Bool, is_block_placed_cb)

    # State Machine Setup
    global SM
    SM = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    rospy.loginfo('sm defined')

    # Initialize state machine variables
    SM.userdata.sm_is_block_placed = False
    SM.userdata.sm_is_pattern_known = False
    SM.userdata.sm_next_block = ''

    # Pattern Globals
    global POSSIBLE_PATTERNS
    POSSIBLE_PATTERNS = create_patterns()

    global STARTER_PATTERNS
    STARTER_PATTERNS = create_starter_patterns()

    with SM:
        # Add states to container
        smach.StateMachine.add('OBSERVE', Observe(),
                               transitions={'examine_puzzle':'EXAMINEPUZZLE'},
                               remapping={'is_block_placed_in':'sm_is_block_placed'})
        smach.StateMachine.add('EXAMINEPUZZLE', ExaminePuzzle(),
                               transitions={'give_next_block':'GIVEBLOCK'},
                               remapping={'is_block_placed_out':'sm_is_block_placed',
                                          'is_pattern_known':'sm_is_pattern_known',
                                          'next_block':'sm_next_block'})
        smach.StateMachine.add('GIVEBLOCK', GiveBlock(),
                               transitions={'observe':'OBSERVE',
                                            'trigger_handover':'TRIGGERHANDOVER'},
                               remapping={'is_pattern_known':'sm_is_pattern_known',
                                          'next_block':'sm_next_block',
                                          'is_block_placed_in':'sm_is_block_placed'})
        # TODO: Change outcome in TRIGGERHANDOVER to the TRIGGERUPDATECHECK
        # state
        smach.StateMachine.add('TRIGGERHANDOVER', ServiceState('handover',
                                                               sia5_hri_fsm.srv.Handover,
                                                               request_slots=['block_pose'],
                                                               response_slots=['handover_bool'],
                                                               outcomes=['observe']),
                               transitions={'observe':'OBSERVE'})
# TODO: Fill in correct initialization variables for service states 4 and 5
# INSERT STATE INTO FLOW: OBSERVE -> TRIGGERPATTERNSTATUS -> EXAMINEPUZZLE
#        smach.StateMachine.add('TRIGGERPATTERNSTATUS', ServiceState('update',
#                                                               sia5_hri_fsm.srv.Update,
#                                                               request_slots=[''],
#                                                               response_slots=['pattern'],
#                                                               outcomes=['examine_puzzle']),
#                               transitions={'examine_puzzle':'EXAMINEPUZZLE'})
# INSERT STATE INTO FLOW: GIVEBLOCK -> TRIGGERUPDATECHECK -> OBSERVE
#        smach.StateMachine.add('TRIGGERUPDATECHECK', ServiceState('update',
#                                                               sia5_hri_fsm.srv.Update,
#                                                               request_slots=[''],
#                                                               response_slots=['pattern'],
#                                                               outcomes=['examine_puzzle']),
#                               transitions={'examine_puzzle':'EXAMINEPUZZLE'})
    sis = smach_ros.IntrospectionServer('sia5_fsm', SM, '/SM_ROOT')
    sis.start()
    SM.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
