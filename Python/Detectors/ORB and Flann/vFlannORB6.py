import cv2
import numpy as np
import pickle
from matplotlib import pyplot as plt

# Load video
cap = cv2.VideoCapture('VID.mp4')

# amount keypoints which will be found
amountKP = 5

# Turn on feature detector ORB
orb = cv2.ORB_create(nfeatures = 5,
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

# Feature matching (FLANN)
index_params = dict(algorithm=1, trees=5)
search_params = dict(check=100)
flann = cv2.FlannBasedMatcher(index_params, search_params)

FrameNumber = 1

while True:
   # Catch frame
   ret, frame = cap.read()

   # condition for break cycle if it was last frame in video
   if ret == False:
      break

   # First frame
   if FrameNumber == 1:
      FrameNumber = FrameNumber + 1

      img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Find descriptors and keypoints in first frame
      kp, des = orb.detectAndCompute(img, None)
      des = np.float32(des)  # change format

      num =[]
      font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
      # Draw keypoints
      img = cv2.drawKeypoints(img, kp, None, color=(0, 0, 255), flags=0)
      # Draw number of keypoints
      for p in range(0, len(kp)):
         num.append(p+1)
         cv2.putText(img, '{}'.format(p + 1), (int(kp[p].pt[0]), int(kp[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                     cv2.LINE_AA)

      # Show result
      # cv2.imshow("Detected keypoints", img)
      plt.imshow(img), plt.title('Detected keypoints')
      while not plt.waitforbuttonpress(): pass

      # For save
      des

      # For save keypoints and numbers on frame
      kp
      num
      print(kp)
      print(num)

      cv2.waitKey(0)

   # Second and following frames
   else:
      img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Find descriptors and keypoints in second frame / train image
      kp1, des1 = orb.detectAndCompute(img1, None)
      des1 = np.float32(des1)  # change format

      # Match 'new' keypoints with 'old'
      matches = flann.knnMatch(des, des1, k=2)

      # Find only good matches
      good_matches = []
      for m, n in matches:
         if m.distance < 0.55 * n.distance:
            good_matches.append(m)
      print(5-len(good_matches))

      kp = []

      num = []
      del_kp1 = []
      see = []

      for m in good_matches:
         kp.append(kp1[m.trainIdx])
         num.append(m.queryIdx+1)
         see.append(m.queryIdx)
         del_kp1.append(m.trainIdx)

      print(see)
      print(del_kp1)

      kp1 = np.delete(kp1, del_kp1, axis=None)



      kp.extend(kp1)


      num.extend(np.arange(des.shape[0]+1, des.shape[0]+1+len(kp1)))

      des1 = np.delete(des1, del_kp1, axis=0)
      des = np.append(des, des1, axis=0)
      print(des.shape[0])


      # Draw keypoints
      img1 = cv2.drawKeypoints(img1, kp, None, color=(0, 0, 255), flags=0)  # trainImage

      # Draw number of keypoints
      font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
      for p in range(0, len(num)):
          cv2.putText(img1, '{}'.format(num[p]), (int(kp[p].pt[0]), int(kp[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                   cv2.LINE_AA)

      # Show result
      # cv2.imshow("Detected keypoints", img1)
      plt.imshow(img1), plt.title('Detected keypoints')
      while not plt.waitforbuttonpress(): pass

      # For save
      kp
      num


      cv2.waitKey(0)

      # else:
      #
      #    # For save 'old' keypoints and their numbers
      #    kp11 = []
      #    kp22 = []
      #    num = []
      #    Mat = []
      #    for mat in good_matches:
      #       kp11.append(kp1[mat.queryIdx])  # found 'old' keypoints on current frame
      #       kp22.append(kp2[mat.trainIdx])  # found 'old' keypoints on current frame
      #       num.append(mat.queryIdx)
      #       Mat.append(mat.trainIdx)
      #       # des1 [mat.queryIdx] = des2[mat.trainIdx]
      #       # kp1[mat.queryIdx] = kp2[mat.trainIdx]
      #
      #    print(Mat)
      #
      #    des2 = np.delete(des2, Mat, axis=0)
      #    des1 = np.append(des1, des2, axis=0)
      #
      #    kp2 = np.delete(kp2, Mat, axis=None)
      #
      #
      #    AmoutOldKeypoints = len(kp1)
      #    for p in range(0, len(kp2)):
      #       kp1.append(kp2[p])
      #       kp22.append(kp2[p])
      #       num.append(AmoutOldKeypoints + p)
      #
      #    # For save
      #    kp = kp22
      #    num
      #
      #    # Draw keypoints
      #    img4 = cv2.drawKeypoints(img2, kp22, None, color=(0, 0, 255), flags=0)  # trainImage
      #
      #    # Draw number of keypoints
      #    font = cv2.FONT_HERSHEY_SIMPLEX  # font for draw numbers keypoints
      #    for p in range(0, len(num)):
      #       cv2.putText(img4, '{}'.format(num[p] + 1), (int(kp22[p].pt[0]), int(kp22[p].pt[1])), font, 0.5, (0, 0, 255), 1,
      #                cv2.LINE_AA)
      #
      #    # Show result
      #    cv2.imshow("Track Keypoints", img4)


      FrameNumber = FrameNumber + 1

      # delay used to frame change
      key = cv2.waitKey(1)
      # exit if user press 'esc'
      if key == 27:
         break

cap.release()
cv2.destroyAllWindows
