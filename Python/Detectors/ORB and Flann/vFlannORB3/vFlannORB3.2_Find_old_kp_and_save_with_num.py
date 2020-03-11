import cv2
import numpy as np
import pickle
from matplotlib import pyplot as plt

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

# Functrions for save and load keypoints and numbers keypoints
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

# Functrions for save numbers and X,Y coordinates of keypoints
def pickle_number_X_Y_keypoints(numbers, keypoints):
   i = 0
   temp_array = []
   for point in keypoints:
      temp = (numbers[i], point.pt[0], point.pt[1])
      i = i + 1
      temp_array.append(temp)
   return temp_array


img1 = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

cap = cv2.VideoCapture('VID.mp4')                     #video frame

# Turn on feature detector ORB
orb = cv2.ORB_create(nfeatures = 500,
                     scaleFactor = 1.2,     # Standart 1.2
                     nlevels = 1,           # Standart 8 --- keypoints not so close 1
                     edgeThreshold = 31,
                     firstLevel = 0,
                     WTA_K = 2,
                     scoreType = 0,         # Harris - 0; FAST - 1. If FAST is used, detector will find more features then 'nfeatures'
                     patchSize = 31,
                     fastThreshold = 20)

# nfeatures	- The maximum number of features to retain.
# scaleFactor - Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
# nlevels - The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels - firstLevel).
# edgeThreshold - This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
# firstLevel - The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
# WTA_K - The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
# scoreType	- The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
# patchSize	- size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
# fastThreshold	- the fast threshold

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


# Feature matching (FLANN)

# FLANN_INDEX_LINEAR = 0,
# FLANN_INDEX_KDTREE = 1,
# FLANN_INDEX_KMEANS = 2,
# FLANN_INDEX_COMPOSITE = 3,
# FLANN_INDEX_KDTREE_SINGLE = 4,
# FLANN_INDEX_HIERARCHICAL = 5,
# FLANN_INDEX_LSH = 6,
# FLANN_INDEX_SAVED = 254,
# FLANN_INDEX_AUTOTUNED = 255,


# index_params = dict(algorithm = 0,
#                    table_number = 6, # 12
#                    key_size = 12,     # 20
#                    multi_probe_level = 1) #2)

index_params = dict(algorithm = 1, trees = 5)
search_params = dict(checks=100)

flann = cv2.FlannBasedMatcher(index_params, search_params)

FrameNumber = 0      # number of frame
temp_array = []      # for save keypoints and numbers keypoints
NumXY_frame = []     # for save numbers and X,Y coordinates of keypoints
while True:
   ret, frame = cap.read()

   # condition for break cycle if it was last frame in video
   if ret == False:
      break
   img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   # Find descriptors and keypoints in train image
   kp2, des2= orb.detectAndCompute(img2, None)

   # Condition for pass all operations and go to next iteration cycle if less two keypoints are found (because in "knnMatch" k=2 below)
   if len(kp2) < 2:
      # Draw and line matches
      img5 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img2)
      # Draw keypoints
      img4 = cv2.drawKeypoints(img2, kp2, None, color=(0, 0, 255), flags=0)  # trainImage

      # Show result
      cv2.imshow("queryImage", img3)
      cv2.imshow("trainImage", img4)
      cv2.imshow("drawMatches", img5)


      # FLANN doesn't work if less two keypoints are found. Therefore there are not keyponts to save
      kp22 = []
      num22 = []

      # Store keypoints and numbers keypoints (for python)
      temp = pickle_keypoints(kp22, num22)
      temp_array.append(temp)

      # Store numbers and X,Y coordinates of keypoints (for matlab)
      NumXY = pickle_number_X_Y_keypoints(num22, kp22)
      NumXY_frame.append(NumXY)

      FrameNumber = FrameNumber + 1

      cv2.waitKey(1)
      continue

   des2 = np.float32(des2)    # change format

   # Match
   matches = flann.knnMatch(des1, des2, k=2)

   # Find only good matches
   good_matches = []
   for m, n in matches:
      if m.distance < 0.55*n.distance:
         good_matches.append(m)

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
   cv2.imshow("trainImage", img4)
   cv2.imshow("drawMatches", img5)

   # plt.imshow(img4), plt.title('trainImage')
   # while not plt.waitforbuttonpress(): pass

   # Numbers keypoints
   num22 = []
   for p in range(0, len(good_matches)):
      num22.append(good_matches[p].queryIdx)
   num22 = np.array(num22)

   print("\n")
   print(kp22)
   print(num22)

   # Store keypoints and numbers keypoints (for python)
   temp = pickle_keypoints_numbers(kp22, num22)
   temp_array.append(temp)

   # Store numbers and X,Y coordinates of keypoints (for matlab)
   NumXY = pickle_number_X_Y_keypoints(num22, kp22)
   NumXY_frame.append(NumXY)

   FrameNumber = FrameNumber + 1

   # delay used to frame change
   key = cv2.waitKey(1)

   # exit if user press 'esc'
   if key == 27:
      break

print(FrameNumber)

# Save for python
pickle.dump(temp_array, open("keypoints_numbers_database.p", "wb"))

# Save for matlab
import scipy.io
scipy.io.savemat('keypoints_numbers_database.mat', mdict={'NumXY_frame': NumXY_frame})



cap.release()
cv2.destroyAllWindows