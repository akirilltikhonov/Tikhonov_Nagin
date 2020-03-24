import cv2
import numpy as np
import pickle
from matplotlib import pyplot as plt

# Load video
cap = cv2.VideoCapture('VID.mp4')

# amount keypoints which will be found
amountKP = 5

# Turn on feature detector ORB
orb = cv2.ORB_create(nfeatures = 50,
                     scaleFactor = 1.2,     # Standart 1.2
                     nlevels = 1,           # Standart 8 --- keypoints not so close 1
                     edgeThreshold = 31,
                     firstLevel = 0,
                     WTA_K = 2,
                     scoreType = 1,         # Harris - 0; FAST - 1. If FAST is used, detector will find more features then 'nfeatures'
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

# Feature matching (FLANN)
index_params = dict(algorithm=1, trees=5)
search_params = dict(check=100)
flann = cv2.FlannBasedMatcher(index_params, search_params)

FrameNumber = 0

while True:
   # Catch frame
   ret, frame = cap.read()

   # condition for break cycle if it was last frame in video
   if ret == False:
      break

   # First frame
   if FrameNumber == 0:
      FrameNumber = FrameNumber + 1

      img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Find descriptors and keypoints in first frame
      kp, des = orb.detectAndCompute(img, None)
      des = np.float32(des)  # change format

      num =[]
      for p in range(0, len(kp)):
            num.append(p + 1)

      # Draw keypoints
      font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
      img = cv2.drawKeypoints(img, kp, None, color=(0, 0, 255), flags=0)

      # Draw number of keypoints
      for p in range(0, len(kp)):
         cv2.putText(img, '{}'.format(num[p]), (int(kp[p].pt[0]), int(kp[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                     cv2.LINE_AA)
      # Show result
      cv2.imshow("First", img)
      # plt.imshow(img), plt.title('Detected keypoints')
      # while not plt.waitforbuttonpress(): pass


      # For save
      des

      # For save keypoints and numbers on frame
      kp
      num
      print(kp)
      print(num)

   # Second and following frames
   else:
      FrameNumber = FrameNumber + 1

      img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Find descriptors and keypoints in second frame / train image
      kp1, des1 = orb.detectAndCompute(img, None)
      des1 = np.float32(des1)  # change format

      # Draw keypoints
      font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
      img1 = cv2.drawKeypoints(img, kp1, None, color=(0, 0, 255), flags=0)

      # Draw number of keypoints
      for p in range(0, len(kp1)):
         cv2.putText(img1, '{}'.format(p+1), (int(kp1[p].pt[0]), int(kp1[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                     cv2.LINE_AA)
      # Show result
      cv2.imshow("kp", img1)
      # plt.imshow(img1), plt.title('Detected keypoints')
      # while not plt.waitforbuttonpress(): pass


      # Match 'new' keypoints with 'old'
      matches = flann.knnMatch(des, des1, k=2)

      # matches - del_kp1 = the worst matches (it is new keypoints)
      del_kp1 = []
      for i in range(0, len(matches)):
         del_kp1.append(matches[i][0].trainIdx)

      # Find only good matches
      good_matches = []
      new_keypoints = []
      for m, n in matches:
         if m.distance < 0.55 * n.distance:
            good_matches.append(m)

      kp = []
      num = []

      for m in good_matches:
         kp.append(kp1[m.trainIdx])
         num.append(m.queryIdx+1)

      kp1 = np.delete(kp1, del_kp1, axis=None)
      kp.extend(kp1)
      num.extend(np.arange(des.shape[0]+1, des.shape[0]+1+len(kp1)))
      des1 = np.delete(des1, del_kp1, axis=0)
      print(des1.shape[0])
      des = np.append(des, des1, axis=0)
      print(des.shape[0])


      # kp_new = []
      # num_new = []
      # des_new = []
      # for m in new_keypoints:
      #    kp_new.append(kp1[m.trainIdx])
      #    des_new.append(des1[m.trainIdx])
         # num_new.append(m.queryIdx + 1)
      # print(des1)
      # print(des_new)
      #
      # print(num)

      # print(np.arange(des.shape[0] + 1, des.shape[0] + 1 + len(kp_new)))
      # num_new = np.arange(des.shape[0] + 1, des.shape[0] + 1 + len(kp_new))
      # print(num_new)

      # kp.extend(kp_new)
      # num.extend(num_new)

      # if len(kp_new) != 0:
      #    des = np.append(des, des_new, axis=0)


      # For save
      kp
      num


   # Draw keypoints
   font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
   img = cv2.drawKeypoints(img, kp, None, color=(0, 0, 255), flags=0)
   # Draw number of keypoints
   for p in range(0, len(kp)):
      cv2.putText(img, '{}'.format(num[p]), (int(kp[p].pt[0]), int(kp[p].pt[1])), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

   # Show result
   cv2.imshow("Detected keypoints", img)
   # plt.imshow(img), plt.title('Detected keypoints')
   # while not plt.waitforbuttonpress(): pass

   # delay used to frame change
   key = cv2.waitKey(0)
   # exit if user press 'esc'
   if key == 27:
      break

cap.release()
cv2.destroyAllWindows
