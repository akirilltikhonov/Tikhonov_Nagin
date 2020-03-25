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

img_all = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

# Load keypoints and descriptors
keypoints_database = pickle.load(open("keypoints_database.p", "rb"))
kp_all, des_all = unpickle_keypoints(keypoints_database[0])
des_all = np.float32(des_all)       # change format

# Draw loaded keypoints
img_all = cv2.drawKeypoints(img_all, kp_all, None, color=(0, 0, 255), flags=0)  # queryImage

# Draw number of keypoints
font = cv2.FONT_HERSHEY_SIMPLEX
for p1 in range(0,  len(kp_all)):
   cv2.putText(img_all, '{}'.format(p1 + 1), (int(kp_all[p1].pt[0]), int(kp_all[p1].pt[1])), font, 0.3, (0, 0, 255), 1, cv2.LINE_AA)

cv2.imshow("queryImage", img_all)




cap = cv2.VideoCapture('VID.mp4')                     #video frame

# Load keypoints and descriptors
keypoints_database = pickle.load(open("keypoints_numbers_database.p", "rb"))

FrameNumber = 0      # number of frame
temp_array = []      # for save keypoints and numbers keypoints
while True:

   ret, frame = cap.read()

   # condition for break cycle if it was last frame in video
   if ret == False:
      break

   # Load keypoints and numbers keypoints
   kp, num = unpickle_keypoints_numbers(keypoints_database[FrameNumber])

   img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   # Draw keypoints
   img1 = cv2.drawKeypoints(img, kp, None, color=(0, 0, 255), flags=0)  # queryImage

   # Draw number of keypoints
   font = cv2.FONT_HERSHEY_SIMPLEX
   for p in range(0, len(kp)):
      Num = num[p]

      cv2.putText(img1, '{}'.format(Num), (int(kp[p].pt[0]), int(kp[p].pt[1])), font, 0.5, (0, 0, 255), 1,
                  cv2.LINE_AA)


   # Show result
   cv2.imshow("Keypoints", img1)
   # plt.imshow(img1), plt.title('trainImage')
   # while not plt.waitforbuttonpress(): pass

   FrameNumber = FrameNumber + 1

   # delay used to frame change
   key = cv2.waitKey(0)

   # exit if user press 'esc'
   if key == 27:
      break

print(FrameNumber)

cap.release()
cv2.destroyAllWindows