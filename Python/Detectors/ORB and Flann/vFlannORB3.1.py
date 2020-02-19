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

# Turn on feature detector ORB
orb = cv2.ORB_create()

# Load keypoints and descriptors
keypoints_database = pickle.load(open("keypoints_database.p", "rb"))
kp1, des1 = unpickle_keypoints(keypoints_database[0])
des1 = np.float32(des1)       # change format

# # Draw loaded keypoints
# img3 = cv2.drawKeypoints(img1, kp1, None, color=(0, 0, 255), flags=0)  # queryImage
#
# # Draw number of keypoints
# font = cv2.FONT_HERSHEY_SIMPLEX
# for p1 in range(0,  len(kp1)):
#    cv2.putText(img3, '{}'.format(p1 + 1), (int(kp1[p1].pt[0]), int(kp1[p1].pt[1])), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
#
# cv2.imshow("queryImage", img3)
#

# Feature matching (FLANN)
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)
FrameNumber = 0
while True:
   ret, frame = cap.read()
   # condition for break cycle if it was last frame in video
   if ret == False:
      break
   img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   # Find descriptors and keypoints in train image
   kp2, des2= orb.detectAndCompute(img2, None)

   # Condition for pass all operations and go to next iteration cycle if less two keypoints are found (because in "knnMatch" k=2 below)
   if len(kp2) <= 1:
      # Draw and line matches
      img5 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img2)
      # Draw keypoints
      img4 = cv2.drawKeypoints(img2, kp2, None, color=(0, 0, 255), flags=0)  # trainImage

      # Show result
      cv2.imshow("queryImage", img3)
      cv2.imshow("trainIdxImage", img4)
      cv2.imshow("img5", img5)

      FrameNumber = FrameNumber + 1
      continue

   des2 = np.float32(des2)    # change format

   # Match
   matches = flann.knnMatch(des1, des2, k=2)

   # Find only good matches
   good_matches = []
   for m, n in matches:
      if m.distance < 0.55*n.distance:
         good_matches.append(m)

   ##
   # print(good_matches)
   # if len(good_matches) == 1:
   #    # print(len(good_matches))
   #    print(good_matches[0].queryIdx)
   # if len(good_matches) == 2:
   #    print(good_matches[0].queryIdx)
   #    print(good_matches[1].queryIdx)
      # print(good_matches[0].trainIdx)
      # print(good_matches[0].distance)
      # print(kp1[good_matches[0].queryIdx])
      # print(kp2[good_matches[0].trainIdx])
      # print(kp1[good_matches[0].queryIdx].pt)
      # print(kp2[good_matches[0].trainIdx].pt)
   ##

   # Draw kepoints
   img5 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img2)

   kp11 = []  # Create list for saving founded old keypoints in queryImage
   kp22 = []  # Create list for saving founded old keypoints in queryImage

   for mat in good_matches:
      kp11.append(kp1[mat.queryIdx])  # save founded old keypoints in queryImage
      kp22.append(kp2[mat.trainIdx])  # save founded old keypoints in trainImage

   # Draw keypoints
   img3 = cv2.drawKeypoints(img1, kp11, None, color=(0, 0, 255), flags=0)  # queryImage
   img4 = cv2.drawKeypoints(img2, kp22, None, color=(0, 0, 255), flags=0)  # trainImage

   # Draw number of keypoints
   font = cv2.FONT_HERSHEY_SIMPLEX
   for p in range(0, len(good_matches)):
      Num = good_matches[p].queryIdx
      cv2.putText(img3, '{}'.format(Num + 1), (int(kp11[p].pt[0]), int(kp11[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                  cv2.LINE_AA)
      cv2.putText(img4, '{}'.format(Num + 1), (int(kp22[p].pt[0]), int(kp22[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                  cv2.LINE_AA)

   # Show result
   cv2.imshow("queryImage", img3)
   cv2.imshow("trainIdxImage", img4)
   cv2.imshow("img5", img5)

   FrameNumber = FrameNumber + 1

   # delay used to frame change
   key = cv2.waitKey(1)
   # exit if user press 'esc'
   if key == 27:
      break
print(FrameNumber)
cap.release()
cv2.destroyAllWindows