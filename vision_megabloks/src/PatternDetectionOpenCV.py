import numpy as np
import imutils
import cv2

cap = cv2.VideoCapture(2)
kernel1 = np.ones((15,15), np.uint8)
kernel2 = np.ones((5,5), np.uint8)

red = []
yellow = []
blue = []
green = []
allPoints = []

# define the list of boundaries
red_boundaries = [([0, 5, 80], [15, 20, 255])]
green_boundaries = [([90, 150, 90], [120, 170, 120])]
blue_boundaries =  [([100, 90, 5], [255, 120, 60])]
yellow_boundaries = [([45, 170, 200], [80, 195, 240])]

def getColors(colorLetter, colorArray, colorArrayPose):
    for (lower, upper) in colorArray:
        # create NumPy arrays from the
        # boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        # find the colors within the specified
        #  boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)
        imageOut = np.hstack([image, output])
        imageOut = cv2.cvtColor(imageOut, cv2.COLOR_BGR2GRAY)
        imageOut = cv2.threshold(imageOut, 10, 255, cv2.THRESH_BINARY)[1]
        imageOut = cv2.morphologyEx(imageOut, cv2.MORPH_CLOSE, kernel1)
        imageOut = cv2.dilate(imageOut,kernel1,iterations=1)
        imageOut = cv2.GaussianBlur(imageOut, (5,5), 0)
        cnts = cv2.findContours(imageOut.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        for c in cnts:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            coords = tuple([colorLetter,cX,cY])
            colorArrayPose.append(coords)
            #cv2.circle(original, (cX,cY), 5, (255,0,0), -1)
            #cv2.imshow('RGB', imageOut)

while True:
    # Capture frame-by-frame
    ret, image = cap.read()
    original = image 
    getColors('b',blue_boundaries,blue)
    
    blue = [i for i in blue if i[1]!=324 and i[2]!=239]
    blue = [i for i in blue if i[2]>=180]
    blue.sort(key=lambda x: x[1]) 

    getColors('r',red_boundaries,red)

    red = [i for i in red if i[1]!=324 and i[2]!=239]
    red = [i for i in red if i[2]>=180]
    red.sort(key=lambda x: x[1]) 

    getColors('y',yellow_boundaries,yellow)

    yellow = [i for i in yellow if i[1]!=324 and i[2]!=239]
    yellow = [i for i in yellow if i[2]>=180]
    yellow.sort(key=lambda x: x[1]) 

    getColors('g',green_boundaries,green)

    green = [i for i in green if i[1]!=324 and i[2]!=239]
    green = [i for i in green if i[2]>=180]
    green.sort(key=lambda x: x[1]) 
    
    allColors = blue + red + yellow + green 
    allColors.sort(key=lambda x:x[1])

    print "order is: "
    for i, (a,b,c) in enumerate(allColors):
        print str(allColors[i][0]) + ", "
    
    del blue[:]
    del red[:]
    del yellow[:]
    del green[:]
    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
