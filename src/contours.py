import numpy as np
import cv2 as cv

im = cv.imread('./test.png')
assert im is not None, "file could not be read, check with os.path.exists()"
imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(imgray, 127, 255, 0)

contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

# FIND CONTOURS: 

# Three arguments: first one is source image, second is contour retrieval mode, third is contour approximation method
# Returns the contours (cords of the boundary of a shape with the same intensity) and heirarchy 
# contours variable is a list of all the contours (represented as a numpy array of (x,y) coordinates of boundary points of the object)

# Third Arg is approximation method where you can either choose all contour cords or the border which saves memory and performence. 

cv.drawContours(imgray, contours, -1, (0,255,0), 3)
area = cv.contourArea(contours)

print(area)

cv.imshow('Image', imgray)
cv.waitKey(0)
cv.destroyAllWindows()