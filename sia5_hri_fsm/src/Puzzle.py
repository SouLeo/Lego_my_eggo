#!/usr/bin/env python
"""
    This code contains the puzzle helper functions to run the
    SIA-5 finite state machine for Dr. Thomaz's HRI Project.

    Maintainer: Selma Wanna, slwanna@utexas.edu
"""
import random
import rospy

def create_patterns():
    """
        Function: create_patterns
        Description: fills out all possible pattern solutions
        Input: none
        Output: all possible solutions
    """
    possible_patterns = []
    possible_patterns.append("rrgygy")
    possible_patterns.append("rryygb")
    possible_patterns.append("rybygg")
    possible_patterns.append("rybyrb")
    possible_patterns.append("yyyyrb")
    possible_patterns.append("yyyygg")
    possible_patterns.append("gbrrby")
    possible_patterns.append("ggrrby")
    possible_patterns.append("bbggry")
    possible_patterns.append("bbggrb")
    possible_patterns.append("bgbgby")
    possible_patterns.append("bgbrby")
    return possible_patterns

def create_starter_patterns():
    """
        Function: create_starter_patterns
        Description: list of blocks people begin with
        Input: none
        Output: list of starting blocks
    """
    starter_patterns = 'rgyb'
    return starter_patterns

def compare_pattern(current_pattern, possible_patterns):
    """
        Function: compare_pattern
        Description: compares currently viewed pattern against possible solutions
        Input: current_pattern (string), possible_patterns (list of strings)
        Output: possible solutions  (list of strings)
    """
    if len(possible_patterns) <= 1:
        rospy.loginfo('Pattern Known from compare_pattern')
        return possible_patterns
    end_index = len(possible_patterns)
    new_possibilities = []
    for i in range(0, end_index):
        if possible_patterns[i].startswith(current_pattern):
            new_possibilities.append(possible_patterns[i])
    return new_possibilities

def next_block(rand, possible_patterns, current_pattern, num_tries=None):
    """
        Function: next_block
        Description: Chooses which block should be tried next
        Input: possible_patterns (string array), current_pattern (string)
        Output: block which should be tried next
    """
    if rand:
        colors = ['r', 'y', 'g', 'b']
        return random.choice(colors)
    else:
        num_blocks = len(current_pattern)
        if num_tries is None:
            first_possible_pattern = possible_patterns[0]
            if num_blocks + 1 <= 6:
                print first_possible_pattern[num_blocks]
                return first_possible_pattern[num_blocks] # return next character
            else:
                print "puzzle complete!"
                print "possible index out of bound"
                rospy.logerr('Indexed out of bound in the next_block() function!!!')
                return '0' # end of sequence. Something went wrong
        else:
            if num_blocks + 1 <= 6:
                print possible_patterns[num_tries % len(possible_patterns)][num_blocks]
                return possible_patterns[num_tries % len(possible_patterns)][num_blocks]
            else:
                print "puzzle complete!"
                print "possible index out of bound"
                rospy.logerr('Indexed out of bound in the next_block() function!!!')
                return '0' # end of sequence. Something went wrong

def is_piece_available(color, starter_pieces):
    """
        Function: is_piece_available
        Description: Checks to see if user has the next block
        Input: next block color (string) and starter pieces (string)
        Output: boolean to determine if robot needs to hand it over
    """
    if color in starter_pieces:
        rospy.loginfo('User has needed piece: ' + color)
        return True
    else:
        rospy.loginfo('Robot will handover piece: ' + color)
        return False

#def main():
#    possible_patterns = []
#    possible_patterns = create_patterns()
#    current_pattern = 'bgbgby'
#    possible_patterns = compare_pattern(current_pattern, possible_patterns)
#    print possible_patterns
#    print next_block(True, possible_patterns, current_pattern, 3)
#if  __name__ == '__main__':
#    main()
