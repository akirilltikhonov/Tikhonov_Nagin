import numpy as np
import cv2


img1 = cv2.imread('1.jpg',0)    # queryImage
img2 = cv2.imread('2.jpg',0)    # trainIdxImage

# Initiate ORB detector
orb = cv2.ORB_create()

# find the keypoints with ORB and compute the descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# Brute Force Matching
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)   # create BFMatcher object (true - delete 'bad' match)
matches = bf.match(des1, des2)                          # Match descriptors
matches = sorted(matches, key = lambda x:x.distance)    # Sort them in the order of their Hamming distance

M = 5           #Numbers of keypoints which will drawed in image
kp11 = []       #Create list for saving M better keypoints in queryImage     
kp22 = []       #Create list for saving M better keypoints in trainImage

for mat in range(0, M):
    kp11.append(kp1[matches[mat].queryIdx])     #save M better keypoints in queryImage
    kp22.append(kp2[matches[mat].trainIdx])     #save M better keypoints in trainImage
    print(kp11)


# Draw M better keypoints and number of keypoints
img3 = cv2.drawKeypoints(img1,kp11,None,color=(0,0,255),flags=0)
img4 = cv2.drawKeypoints(img2,kp22,None,color=(0,0,255),flags=0)

print(kp11[0].pt)
print(int(kp11[0].pt[0]))
print(int(kp11[0].pt[1]))
print(kp11[0].size)

# Draw number of keypoints
font = cv2.FONT_HERSHEY_SIMPLEX
for p in range(0, M):
    cv2.putText(img3,'{}'.format(p+1),(int(kp11[p].pt[0]),int(kp11[p].pt[1])), font, 1,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(img4,'{}'.format(p+1),(int(kp22[p].pt[0]),int(kp22[p].pt[1])), font, 1,(0,0,255),2,cv2.LINE_AA)

# Show result
cv2.imshow('1',img3)
cv2.imshow('2',img4)

matching_result = cv2.drawMatches(img1, kp1, img2, kp2, matches[:M], None, flags=2)
cv2.imshow("Matching result", matching_result)

cv2.waitKey(0)
cv2.destroyAllWindows()


