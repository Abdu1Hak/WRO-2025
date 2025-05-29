import numpy as np
import cv2 as cv
 
img = cv.imread('./messi5.jpg')
assert img is not None, "file could not be read, check with os.path.exists()"

px = img[100,100]
# ball = img[280:340, 330:390]
# img[273:333, 100:160] = ball

print(px)
print(img.shape)
print(img.size)
print(img.dtype)

# b,g,r = cv.split(img)
img[:,:,2] = 0
# img = cv.merge((b,g,r))

cv.imshow('Image', img)
cv.waitKey(0)
cv.destroyAllWindows()