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

#img1 = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

# Load video
cap = cv2.VideoCapture('VID.mp4')

# amount keypoints which will be found
amountKP = 2

# Feature matching (FLANN)
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

FrameNumber = 0
kp = []
des = []

while True:
   # Turn on feature detector ORB
   orb = cv2.ORB_create(amountKP)

   # Catch frame
   ret, frame = cap.read()

   # condition for break cycle if it was last frame in video
   if ret == False:
      break

   if FrameNumber == 0:
      FrameNumber = FrameNumber+ 1
      img1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      # Find descriptors and keypoints in first frame
      kp1, des1 = orb.detectAndCompute(img1, None)
      des1 = np.float32(des1)  # change format

   else:
      img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      # Find descriptors and keypoints in second frame / train image
      kp2, des2 = orb.detectAndCompute(img2, None)

      des2 = np.float32(des2)  # change format

      print(kp2)
      print(des2)

      # Match
      matches = flann.knnMatch(des1, des2, k=2)

      # Find only good matches
      good_matches = []
      for m, n in matches:
         if m.distance < 0.55 * n.distance:
            good_matches.append(m)


      if len(good_matches) == amountKP:

         kp11 = []
         kp22 = []
         des22 = np.zeros((amountKP, 32))

         for mat in good_matches:
            kp11.append(kp1[mat.queryIdx])  # save founded old keypoints in queryImage
            kp22.append(kp2[mat.trainIdx])  # save founded old keypoints in trainImage
            des22[mat.trainIdx] = des2[mat.trainIdx]
            # print(mat.queryIdx)
            # print(mat.trainIdx)

         # print(des22.dtype)

         # Keypoints and descriptors in train image for next frame (for next query image)
         kp1 = kp22
         des1 = np.float32(des22)
         print(kp1)
         print(des1)
         print('\n')

         # Numbers keypoints
         num1 = []
         for p in range(0, len(good_matches)):
            num1.append(good_matches[p].queryIdx)

         # For save
         kp = kp1
         num = np.array(num1)
         # print(kp)
         # print(num)

      else:

         addKP = amountKP - len(good_matches)

         kp11 = []
         kp22 = []

         for mat in good_matches:
            kp11.append(kp1[mat.queryIdx])  # save founded old keypoints in queryImage
            kp22.append(kp2[mat.trainIdx])  # save founded old keypoints in trainImage
            des1 [mat.trainIdx] = des2[mat.trainIdx]
            # print(mat.queryIdx)
            # print(mat.trainIdx)

         # Keypoints and descriptors in train image for next frame (for next query image)
         kp1 = kp1
         print(kp1)
         des1 = np.float32(des1)
         print(des1)

         # For save
         kp = kp22
         num = num
         # print(kp)
         # print(num)

         orb = cv2.ORB_create(addKP)




      # Draw keypoints
      img3 = cv2.drawKeypoints(img1, kp11, None, color=(0, 0, 255), flags=0)  # queryImage
      img4 = cv2.drawKeypoints(img2, kp22, None, color=(0, 0, 255), flags=0)  # trainImage

      # # Draw number of keypoints
      # font = cv2.FONT_HERSHEY_SIMPLEX
      # for p in range(0, len(good_matches)):
      #    Num = good_matches[p].queryIdx
      #    cv2.putText(img3, '{}'.format(Num + 1), (int(kp11[p].pt[0]), int(kp11[p].pt[1])), font, 0.5, (0, 0, 255), 1,
      #                cv2.LINE_AA)
      #    cv2.putText(img4, '{}'.format(Num + 1), (int(kp22[p].pt[0]), int(kp22[p].pt[1])), font, 0.5, (0, 0, 255), 1,
      #                cv2.LINE_AA)

      # Draw matches
      img5 = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img2)

      # Show result
      cv2.imshow("queryImage", img3)
      cv2.imshow("trainIdxImage", img4)
      cv2.imshow("img5", img5)

      FrameNumber = FrameNumber + 1

      # delay used to frame change
      key = cv2.waitKey(0)
      # exit if user press 'esc'
      if key == 27:
         break

cap.release()
cv2.destroyAllWindows


