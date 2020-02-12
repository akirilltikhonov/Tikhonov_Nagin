import numpy as np
import cv2
import video


if __name__ == '__main__':
   #make window with name 'original'
   cv2.namedWindow( "Top 'k' features", cv2.WINDOW_NORMAL)
   cv2.waitKey(500)
   
   # make object 'cap' to capture  video frame
   cap = cv2.VideoCapture('VID.mp4')

   # width, height and fps in video
   w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));
   h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));
   fps = cap.get(cv2.CAP_PROP_FPS);
        
   # Define the codec and create VideoWriter object
   fourcc = cv2.VideoWriter_fourcc(*'mp4v')
   #out = cv2.VideoWriter('VID-1.mp4', fourcc, fps, (w, h))
   
   while True:

      # catch current frame and put it to variable 'img'
      flag, img = cap.read()
      
      
      # condition for break cycle if it was last frame in video
      if flag == False:
         break

      gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
      corners = cv2.goodFeaturesToTrack(gray, maxCorners=5, qualityLevel=0.05,
      minDistance=25)

      
      for item in corners:
          x, y = item[0]
          cv2.circle(img, (x,y), 5, 255, -1)
      cv2.imshow("Top 'k' features", img)

      #cv2.waitKey(0)
     
     
      # write\save new frame
      #out.write(img)
  
 
      # delay used to frame change     
      ch = cv2.waitKey(1)
      # exit if user press 'esc'
      if ch == 27:
         break

   cap.release()
   #out.release()
   cv2.destroyAllWindows()
