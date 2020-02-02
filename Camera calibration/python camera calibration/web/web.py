import cv2
import video

if __name__ == '__main__':
   # make window with name 'oroginal'
   cv2.namedWindow( "original" )

   # make object 'cap' to capture  camera's frame
   cap = video.create_capture(0)

   while True:
      # catch current frame and put it to variable 'img'
      flag, img = cap.read()
      try:
         # display frame in window with name 'original'
         cv2.imshow('original', img)
      except:
         # close camera's object    
         cap.release()
         raise

      ch = cv2.waitKey(5)
      # exit if user press 'esc'
      if ch == 27:
         break

   #
   cap.release()
   cv2.destroyAllWindows()
