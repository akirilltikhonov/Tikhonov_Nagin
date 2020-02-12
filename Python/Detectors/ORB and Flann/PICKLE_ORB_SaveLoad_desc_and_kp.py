import cv2
import numpy as np
import pickle

def pickle_keypoints(keypoints, descriptors):
    i = 0
    temp_array = []
    for point in keypoints:
        temp = (point.pt, point.size, point.angle, point.response, point.octave,
        point.class_id, descriptors[i])
        i = i + 1
        temp_array.append(temp)
    return temp_array

def unpickle_keypoints(array):
    keypoints = []
    descriptors = []
    for point in array:
        temp_feature = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5])
        temp_descriptor = point[6]
        keypoints.append(temp_feature)
        descriptors.append(temp_descriptor)
    return keypoints, np.array(descriptors)


img = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage
cv2.imshow("original", img)
#key = cv2.waitKey(0)

# Features
orb = cv2.ORB_create(50)
kp1, des1 = orb.detectAndCompute(img, None)

img1 = cv2.drawKeypoints(img, kp1, img)
cv2.imshow("for save", img1)

#Store keypoint features
temp_array = []
temp = pickle_keypoints(kp1, des1)
temp_array.append(temp)
pickle.dump(temp_array, open("keypoints_database.p", "wb"))

# delay
#key = cv2.waitKey(0)

# Retrieve Keypoint Features
keypoints_database = pickle.load(open("keypoints_database.p", "rb"))
kp2, des2 = unpickle_keypoints(keypoints_database[0])

print(des1)
print('\n')
print(des2)

print(kp1)
print(kp2)

img2 = cv2.drawKeypoints(img, kp2, img)
cv2.imshow("load", img2)

cv2.waitKey(0)
cv2.destroyAllWindows