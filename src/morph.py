
# EROSION - all pixels near boyndary will be discarded depending on size of kernel - (Removes SMALL white noises)

import cv2 
import numpy as np
import os
print("Current Working Directory:", os.getcwd()) 

while True: 
  img = cv2.imread('src/kernel.png')
  assert img is not None, "file could not be read, check with os.path.exists()"

  # structuring element (kernel) - decides the nature of operation
  # higher kernel number means erode more
  kernel = np.ones((5,5),np.uint8)

  # Remove white noise
  erosion = cv2.erode(img,kernel,iterations = 1)

  # Dilate the shrink object - after removing all bounding pixels of 1 to 0 - the resulting object is dilated
  dilation = cv2.dilate(erosion, kernel, iterations=1)

  cv2.imshow("original_img", img)
  cv2.imshow("erosion", erosion)
  cv2.imshow("erosion+dilation", dilation)

  k = cv2.waitKey(60)
  if k == ord('q'):
    break

cv2.destroyAllWindows()