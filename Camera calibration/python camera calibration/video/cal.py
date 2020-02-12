import numpy as np
import cv2
import video

# load matrix intrisinc parametrs and distortional coefficient
mtx = np.load('mtx.npy')
dist = np.load('dist.npy')


if __name__ == '__main__':
   #make window with name 'original'
   cv2.namedWindow( "original", cv2.WINDOW_NORMAL)
   cv2.namedWindow( "undistorted", cv2.WINDOW_NORMAL)
   cv2.waitKey(500)
   
   # make object 'cap' to capture  video frame
   cap = cv2.VideoCapture('VID2.mp4')

   # width, height and fps in video
   w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));
   h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));
   fps = cap.get(cv2.CAP_PROP_FPS);
        
   # Define the codec and create VideoWriter object
   fourcc = cv2.VideoWriter_fourcc(*'mp4v')
   out = cv2.VideoWriter('VID2-1.mp4', fourcc, fps, (w, h))
   
   while True:

      # catch current frame and put it to variable 'img'
      flag, img = cap.read()
      
      # condition for break cycle if it was last frame in video
      if flag == False:
         break

      # Determine windth and height frame, formation new matrix
      # of intrisinc parametrs and ROI for crop image
      #h,  w = img.shape[:2]     #(determine in beginning)
      newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h))
      

      # undistorted frame\image
      dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

      # crop the image (if use it video can not be saved)
      #x,y,w,h = roi
      #dst = dst[y:y+h, x:x+w]

      # write\save the undistorted frame
      out.write(dst)
   
      # display undistorted frame in window with name 'undistorted'
      cv2.imshow('undistorted', dst)

      # display frame in window with name 'undistortion'
      cv2.imshow('original', img)
      

      # delay used to frame change     
      ch = cv2.waitKey(1)
      # exit if user press 'esc'
      if ch == 27:
         break

   cap.release()
   out.release()
   cv2.destroyAllWindows()
