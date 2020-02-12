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

img = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

cap = cv2.VideoCapture('VID.mp4')                     #video frame

# Features
orb = cv2.ORB_create()
# Load keypoints and descriptors
keypoints_database = pickle.load(open("keypoints_database.p", "rb"))
kp_image, desc_image = unpickle_keypoints(keypoints_database[0])
desc_image = np.float32(desc_image)       # change format

# Feature matching
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
   ret, frame = cap.read()
   grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   kp_grayframe, des_grayframe = orb.detectAndCompute(grayframe, None)
   des_grayframe = np.float32(des_grayframe)    # change format

   matches = flann.knnMatch(desc_image, des_grayframe, k=2)

   good_points = []
   for m, n in matches:
      if m.distance < 0.55*n.distance:
         good_points.append(m)

   img3 = cv2.drawMatches(img, kp_image, grayframe, kp_grayframe, good_points, grayframe)

   #cv2.imshow("query", img)
   #cv2.imshow("train", grayframe)
   cv2.imshow("img3", img3)


   # delay used to frame change
   key = cv2.waitKey(1)
   # exit if user press 'esc'
   if key == 27:
      break

cap.release()
cv2.destroyAllWindows