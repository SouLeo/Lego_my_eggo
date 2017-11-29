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

def nextBlock(possiblePatterns, currentPattern):
    # Function: nextBlock
    # Description: Chooses which block should be tried next
    # Input: currentPattern (string)
    # Output: block which should be tried next
    firstpossiblePattern = possiblePatterns[0]
    numBlocks = len(currentPattern)
    if (numBlocks + 1 <= 6):
        print firstpossiblePattern[numBlocks]
        return firstpossiblePattern[numBlocks + 1] # return next character
    else:
        print "index out of bound"
        rospy.logerr('Indexed out of bound in the nextBlock() function!!!')
        return '0' # end of sequence. Something went wrong
'''
def main():
    possiblePatterns = []
    possiblePatterns = createPatterns()
    currentPattern = 'rr'
    possiblePatterns = comparePattern(currentPattern, possiblePatterns)
    print possiblePatterns 
    nextBlock(possiblePatterns, currentPattern)

if  __name__ == '__main__':
    main()
'''
