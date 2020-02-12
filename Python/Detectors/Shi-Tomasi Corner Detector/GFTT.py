import cv2
import numpy as np

img = cv2.imread('3.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

corners = cv2.goodFeaturesToTrack(gray, maxCorners=5, qualityLevel=0.05,
minDistance=25)

corners = np.float32(corners)
print(corners)

for item in corners:
    x, y = item[0]
    cv2.circle(img, (x,y), 5, 255, -1)
cv2.imshow("Top 'k' features", img)
cv2.waitKey()
