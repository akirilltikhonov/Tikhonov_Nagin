import cv2
import numpy as np

img = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage

cap = cv2.VideoCapture('VID.mp4')

# Features
orb = cv2.ORB_create()
kp_image, desc_image = orb.detectAndCompute(img, None)
#img = cv2.drawKeypoints(img, kp_image, img)

# Feature matching
index_params = dict(algorithm=0, trees=5)
search_params = dict()
flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
   ret, frame = cap.read()
   grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #trainImage

   kp_grayframe, desc_grayframe = orb.detectAndCompute(grayframe, None)

   desc_image = np.float32(desc_image)
   desc_grayframe = np.float32(desc_grayframe)

   matches = flann.knnMatch(desc_image, desc_grayframe, k=2)

   good_points = []
   for m, n in matches:
      if m.distance < 0.5*n.distance:
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