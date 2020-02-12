import cv2
import sys
import os
for path in sys.path:
   if os.path.exists(os.path.join(path, 'cv2')):
      print('cv2 is here: {}'.format(path))
