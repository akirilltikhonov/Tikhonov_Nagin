import cv2
import numpy as np
import pickle

# Functrions for save and load keypoints and descriptors
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
      temp_feature = cv2.KeyPoint(x=point[0][0], y=point[0][1], _size=point[1], _angle=point[2], _response=point[3],
                                  _octave=point[4], _class_id=point[5])
      temp_descriptor = point[6]
      keypoints.append(temp_feature)
      descriptors.append(temp_descriptor)
   return keypoints, np.array(descriptors)

img1 = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

cap = cv2.VideoCapture('VID.mp4')                     #video frame

# Features
orb = cv2.ORB_create(2)
# Load keypoints and descriptors
keypoints_database = pickle.load(open("keypoints_database.p", "rb"))
kp1, des1 = unpickle_keypoints(keypoints_database[0])
des1 = np.float32(des1)       # change format

# Feature matching (FLANN)
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
   ret, frame = cap.read()
   img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   kp2, des2= orb.detectAndCompute(img2, None)
   des2 = np.float32(des2)    # change format

   matches = flann.knnMatch(des1, des2, k=2)

   cv2.waitKey(0)

   #print(kp1)
   #print(kp2)
   #print(des1)
   #print(des2)

   print(matches)
   print('\n')
   print(matches[0])
   print('\n')
   #print(matches[0].queryIdx)

   good_matches = []
   for m, n in matches:
      if m.distance < 0.55*n.distance:
         good_matches.append(m)
   print(good_matches)
   if good_matches != []:
      print(good_matches[0].queryIdx)
      print(good_matches[0].trainIdx)
      print(good_matches[0].distance)
      print(kp1[good_matches[0].queryIdx])
      print(kp2[good_matches[0].trainIdx])
      print(kp1[good_matches[0].queryIdx].pt)
      print(kp2[good_matches[0].trainIdx].pt)


   img3 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img2)


   #cv2.imshow("query", img1)
   #cv2.imshow("train", img2)
   cv2.imshow("img3", img3)

   # delay used to frame change
   key = cv2.waitKey(1)
   # exit if user press 'esc'
   if key == 27:
      break

cap.release()
cv2.destroyAllWindows