import numpy as np
import cv2
import glob

# load matrix intrisinc parametrs and distortional coefficient
mtx = np.load('mtx.npy')
dist = np.load('dist.npy')

print(mtx)
print(dist)

#mtx = np.zeros((3,3), np.float32)
#dist = np.zeros((1,5), np.float32) 


#dist[0,0] = -0.2

print(mtx)
print(dist)

Num = 1
images = glob.glob('photo before and after calibration\*.jpg')
for fname in images:

    img = cv2.imread(fname)

    # Determine windth and height frame, formation new matrix
    # of intrisinc parametrs and ROI for crop image   
    h,  w = img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    

    # undistorted frame
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    # x,y,w,h = roi
    # dst = dst[y:y+h, x:x+w]
	
    cv2.imwrite('photo before and after calibration/({})-4.jpg'.format(Num),dst)
    Num = Num + 1
  

