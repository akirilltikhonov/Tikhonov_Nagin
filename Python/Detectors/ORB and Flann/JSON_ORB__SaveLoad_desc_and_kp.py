import cv2
import numpy as np
import json

img = cv2.imread("query.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage
cv2.imshow("original", img)
key = cv2.waitKey(0)

# Features
orb = cv2.ORB_create()
kp_image, desc_image = orb.detectAndCompute(img, None)

img1 = cv2.drawKeypoints(img, kp_image, img)
cv2.imshow("for save", img1)

def save_2_jason(arr, filename):
    data = {}
    cnt = 0
    for i in arr:
        data['KeyPoint_%d' % cnt] = []
        data['KeyPoint_%d' % cnt].append({'x': i.pt[0]})
        data['KeyPoint_%d' % cnt].append({'y': i.pt[1]})
        data['KeyPoint_%d' % cnt].append({'size': i.size})
        cnt += 1
    with open('{}.txt'.format(filename), 'w') as outfile:
        json.dump(data, outfile)

filename1 = 'data'
save_2_jason(kp_image, filename1)

# delay
key = cv2.waitKey(0)

def read_from_jason(filename):
    result = []
    with open('{}.txt'.format(filename)) as json_file:
        data = json.load(json_file)
        cnt = 0
        while (data.__contains__('KeyPoint_%d' % cnt)):
            pt = cv2.KeyPoint(x=data['KeyPoint_%d' % cnt][0]['x'], y=data['KeyPoint_%d' % cnt][1]['y'],
                              _size=data['KeyPoint_%d' % cnt][2]['size'])
            result.append(pt)
            cnt += 1
    return result

filename2 = 'data'
kp_read_from_jason = read_from_jason(filename2)

img2 = cv2.drawKeypoints(img, kp_read_from_jason, img)
cv2.imshow("load", img2)

print(kp_image)
print(kp_read_from_jason)

cv2.waitKey(0)
cv2.destroyAllWindows