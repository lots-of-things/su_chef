import numpy as np
import cv2
im = cv2.imread('/home/stedn/Downloads/test_contour_fit.png')
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
contours, im2 = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

ellipse = cv2.fitEllipse(contours[28])
centerE = ellipse[0]
# Gets rotation of ellipse; same as rotation of contour
rotation = ellipse[2]
# Gets width and height of rotated ellipse
widthE = ellipse[1][0]
heightE = ellipse[1][1]
# Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
# rotation = translateRotation(rotation, widthE, heightE)

cv2.ellipse(im, ellipse, (23, 184, 80), 3)
cv2.imwrite('output.png', im)