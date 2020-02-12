import numpy as np
import cv2



if __name__ == '__main__':
   #make window with name 'original'
   #cv2.namedWindow( "Top 'k' features", cv2.WINDOW_NORMAL)
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

  
   first = 0
   while True:

      # catch current frame and put it to variable 'img'
      flag, img1 = cap.read()
            
      # condition for break cycle if it was last frame in video
      if flag == False:
         break

      if first == 1:
         # Initiate ORB detector
         orb = cv2.ORB_create()
         # find the keypoints with ORB and compute the descriptors with ORB
         kp1, des1 = orb.detectAndCompute(img1, None)
         kp2, des2 = orb.detectAndCompute(img2, None)

         # Brute Force Matching
         bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)   # create BFMatcher object (true - delete 'bad' match)
         matches = bf.match(des1, des2)                          # Match descriptors
         matches = sorted(matches, key = lambda x:x.distance)    # Sort them in the order of their Hamming distance

         M = 5           #Numbers of keypoints which will drawed in image
         kp11 = []       #Create list for saving M better keypoints in queryImage     
         kp22 = []       #Create list for saving M better keypoints in trainImage

         for mat in range(0, M):
            kp11.append(kp1[matches[mat].queryIdx])     #save M better keypoints in queryImage
            kp22.append(kp2[matches[mat].trainIdx])     #save M better keypoints in trainImage
         cv2.waitKey(0)

         # Draw M better keypoints and number of keypoints
         img3 = cv2.drawKeypoints(img1,kp11,None,color=(0,0,255),flags=0)
         img4 = cv2.drawKeypoints(img2,kp22,None,color=(0,0,255),flags=0)
         
         # Draw number of keypoints
         font = cv2.FONT_HERSHEY_SIMPLEX
         for p in range(0, M):
            cv2.putText(img3,'{}'.format(p+1),(int(kp11[p].pt[0]),int(kp11[p].pt[1])),font,0.5,(0,0,255),1,cv2.LINE_AA)
            cv2.putText(img4,'{}'.format(p+1),(int(kp22[p].pt[0]),int(kp22[p].pt[1])),font,0.5,(0,0,255),1,cv2.LINE_AA)

         # Show result
         cv2.imshow("queryImage", img3)
         cv2.imshow("trainIdxImage", img4)
         
         #
         img2 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
         
      else:
         first = 1
         img2 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
      
      
      
      
      cv2.waitKey(0)
     
     
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
