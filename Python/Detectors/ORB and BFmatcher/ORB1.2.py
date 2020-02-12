import numpy as np
import cv2


img1 = cv2.imread('3.jpg',0)
img2 = cv2.imread('4.jpg',0)

# Initiate STAR detector
orb = cv2.ORB_create()

# find the keypoints with ORB and compute the descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

kp1 = kp1[0:5]
kp2 = kp2[0:5]

# draw only keypoints location,not size and orientation
img3 = cv2.drawKeypoints(img1,kp1,None,color=(0,255,0), flags=0)
img4 = cv2.drawKeypoints(img2,kp2,None,color=(0,255,0), flags=0)

cv2.imshow('1',img3)
cv2.imshow('2',img4)
cv2.waitKey(0) 


