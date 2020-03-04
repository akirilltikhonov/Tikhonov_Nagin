import cv2
import numpy as np
import pickle

# Functrions for save and load keypoints and their numbers
def pickle_keypoints_numbers(keypoints, numbers):
   i = 0
   temp_array = []
   for point in keypoints:
      temp = (point.pt, point.size, point.angle, point.response, point.octave,
              point.class_id, numbers[i])
      i = i + 1
      temp_array.append(temp)
   return temp_array
def unpickle_keypoints_numbers(array):
   keypoints = []
   numbers = []
   for point in array:
      temp_feature = cv2.KeyPoint(x=point[0][0], y=point[0][1], _size=point[1], _angle=point[2], _response=point[3],
                                  _octave=point[4], _class_id=point[5])
      temp_number = point[6]
      keypoints.append(temp_feature)
      numbers.append(temp_number)
   return keypoints, np.array(numbers)

img1 = cv2.imread("query1.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage1
img2 = cv2.imread("query2.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage2

# Turn on feature detector ORB
orb = cv2.ORB_create(2)

# Find descriptors and keypoints in train image
kp1, des1= orb.detectAndCompute(img1, None)
kp2, des2= orb.detectAndCompute(img2, None)
print(kp1)
print(kp2)

img3 = cv2.drawKeypoints(img1, kp1, img1)
img4 = cv2.drawKeypoints(img2, kp2, img2)
cv2.imshow("3", img3)
cv2.imshow("4", img4)

num1 = [1,2]
num2 = [3,4]
num1 = np.array(num1)
num2 = np.array(num2)
print(num1)
print(num2)

#Store keypoint features
temp_array = []
temp = pickle_keypoints_numbers(kp1, num1)
temp_array.append(temp)
temp = pickle_keypoints_numbers(kp2, num2)
temp_array.append(temp)
pickle.dump(temp_array, open("keypoints_numbers_database_test.p", "wb"))

# Retrieve Keypoint Features
keypoints_database = pickle.load(open("keypoints_numbers_database_test.p", "rb"))
kp11, num11 = unpickle_keypoints_numbers(keypoints_database[0])
kp22, num22 = unpickle_keypoints_numbers(keypoints_database[1])

print('\n')
print(num11)
print(num22)
img5 = cv2.drawKeypoints(img1, kp1, img1)
img6 = cv2.drawKeypoints(img2, kp2, img2)
cv2.imshow("5", img5)
cv2.imshow("6", img6)

cv2.waitKey(0)
cv2.destroyAllWindows