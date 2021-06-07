import cv2 as cv
import numpy as np

# The below line is for making our own blank image
blank = np.zeros((500, 500, 3), dtype='uint8')
cv.imshow('Blank', blank)

# The below line is for importing an image
# img = cv.imread('images/Cartoon Forest.jpg')
# cv.imshow('Forest', img)

# paint the image a color
# blank[200:300, 300:400] = 0, 255, 0
# cv.imshow('Green', blank)

# draw a rectangle
# Order of parameters:
#   image, coord1, coord2, color, thickness of border
#   coordinates can have either numerical values or relative values to the size of the image
cv.rectangle(blank, (0, 0), (blank.shape[1]//2, blank.shape[0]//2), (0, 255, 0), thickness=-1)
cv.imshow('Rectangle', blank)

# draw a circle
# parameters required:
#   image, center of circle, radius, color, thickness of border
cv.circle(blank, (250, 250), 50, (0, 0, 255), thickness=4)
cv.imshow('Circle', blank)

# draw a line
# parameters are the same as the rectangle
cv.line(blank, (0, 0), (blank.shape[1]//2, blank.shape[0]//2), (255, 0, 0), thickness=3)
cv.imshow('Line', blank)

# write text
# parameters required:
#   image, text string, coordinates, font type, font scale, color, thickness
cv.putText(blank, 'Yo wassup?', (20, 225), cv.FONT_HERSHEY_DUPLEX, 1.5, (255, 255, 0), thickness=2)
cv.imshow('Text', blank)

cv.waitKey(0)