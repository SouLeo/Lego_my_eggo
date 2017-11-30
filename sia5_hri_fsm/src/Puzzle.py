#!/usr/bin/env python

# Maintainer: Selma Wanna, slwanna@utexas.edu
import roslib
import rospy

def createPatterns():
    # Function: createPatterns
    # Description: fills out all possible pattern solutions
    # Input: none
    # Output: all possible solutions
    possiblePatterns = []
    possiblePatterns.append("rrgygy")
    possiblePatterns.append("rryygb")
    possiblePatterns.append("rybygg")
    possiblePatterns.append("rybyrb")
    possiblePatterns.append("yyyyrb")
    possiblePatterns.append("yyyygg")
    possiblePatterns.append("gbrrby")
    possiblePatterns.append("ggrrby")
    possiblePatterns.append("bbggry")
    possiblePatterns.append("bbggrb")
    possiblePatterns.append("bgbgby")
    possiblePatterns.append("bgbrby")
    return possiblePatterns

def comparePattern(currentPattern, possiblePatterns):
    # Function: comparePattern
    # Description: compares currently viewed pattern against possible solutions
    # Input: currentPattern (string), possiblePatterns (list of strings)
    # Output: possible solutions  (list of strings)
    if (len(possiblePatterns) <= 1):
        rospy.loginfo('Pattern Known from comparePattern')
        return possiblePatterns 
    endIndex = len(possiblePatterns)
    newPossibliities = []
    for i in range (0, endIndex):
        if possiblePatterns[i].startswith(currentPattern):
            newPossibliities.append(possiblePatterns[i])
    return newPossibliities

def nextBlock(possiblePatterns, currentPattern, numTries = None):
    # TODO: check to see if user has a block in his/her workspace. i.e.
    # do we need to go to give block?
    
    # Function: nextBlock
    # Description: Chooses which block should be tried next
    # Input: possiblePatterns (string array), currentPattern (string)
    # Output: block which should be tried next
    numBlocks = len(currentPattern)
    if numTries is None:
        firstpossiblePattern = possiblePatterns[0]
        if (numBlocks + 1 <= 6):
            print firstpossiblePattern[numBlocks]
            return firstpossiblePattern[numBlocks] # return next character
        else:
            print "puzzle complete!"
            print "possible index out of bound"
            rospy.logerr('Indexed out of bound in the nextBlock() function!!!')
            return '0' # end of sequence. Something went wrong
    else:
        if (numBlocks + 1 <= 6):
            print possiblePatterns[numTries % len(possiblePatterns)][numBlocks]
            return possiblePatterns[numTries % len(possiblePatterns)][numBlocks]
        else:
            print "puzzle complete!"
            print "possible index out of bound"
            rospy.logerr('Indexed out of bound in the nextBlock() function!!!')
            return '0' # end of sequence. Something went wrong
'''    
def main():
    possiblePatterns = []
    possiblePatterns = createPatterns()
    currentPattern = 'bgbgby'
    possiblePatterns = comparePattern(currentPattern, possiblePatterns)
    print possiblePatterns 
    nextBlock(possiblePatterns, currentPattern, 3)

if  __name__ == '__main__':
    main()
'''
