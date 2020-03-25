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
        temp_feature = cv2.KeyPoint(x=point[0][0],y=point[0][1],_size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5])
        temp_descriptor = point[6]
        keypoints.append(temp_feature)
        descriptors.append(temp_descriptor)
    return keypoints, np.array(descriptors)

img1 = cv2.imread("query1.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage1
img2 = cv2.imread("query2.jpg", cv2.IMREAD_GRAYSCALE)   #queryImage2

# Features
orb = cv2.ORB_create(
                     nfeatures = 1,
                     scaleFactor = 1.2,     # Standart 1.2
                     nlevels = 1,           # Standart 8 --- keypoints not so close 1
                     edgeThreshold = 31,
                     firstLevel = 0,
                     WTA_K = 2,
                     scoreType = 0,         # Harris - 0; FAST - 1. If FAST is used, detector will find more features then 'nfeatures'
                     patchSize = 31,
                     fastThreshold = 20,
                     )

# nfeatures	- The maximum number of features to retain.
# scaleFactor - Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
# nlevels - The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels - firstLevel).
# edgeThreshold - This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
# firstLevel - The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
# WTA_K - The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
# scoreType	- The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
# patchSize	- size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
# fastThreshold	- the fast threshold

# Initialization of mask (255 - search keypoints, 0 - don't search keypoints)
mask1 = np.full((img1.shape[0], img1.shape[1]), 255, dtype=np.uint8)
mask2 = np.full((img2.shape[0], img2.shape[1]), 255, dtype=np.uint8)

amountKP = 100      # amount keypoints that we need to find
dist_kp = 8        # distance beetwen keypoints (min dist_kp = 1)


kp1 = []
kp2 = []
kp = []

# First image
while len(kp1) < amountKP:

    # Find descriptors and keypoints using a mask
    kp, des = orb.detectAndCompute(img1, mask1)
    des = np.float32(des)

    # Correction of mask (255 - search keypoints, 0 - don't search keypoints)
    mask1[int(kp[0].pt[1])-dist_kp : int(kp[0].pt[1])+dist_kp, int(kp[0].pt[0])-dist_kp : int(kp[0].pt[0])+dist_kp] = 0

    # Write keypoints and descriptors that are not so close

    if len(kp1) == 0:
        des1 = des
    else:
        des1 = np.append(des1, des, axis=0)

    kp1.append(kp[0])
    # kp1 = np.append(kp1, kp, axis=0)

# Second image
while len(kp2) < amountKP:

    # Find descriptors and keypoints using a mask
    kp, des = orb.detectAndCompute(img2, mask2)
    des = np.float32(des)

    # Correction of mask (255 - search keypoints, 0 - don't search keypoints)
    mask2[int(kp[0].pt[1])-dist_kp : int(kp[0].pt[1])+dist_kp, int(kp[0].pt[0])-dist_kp : int(kp[0].pt[0])+dist_kp] = 0

    # Write keypoints and descriptors that are not so close
    if len(kp2) == 0:
        des2 = des
    else:
        des2 = np.append(des2, des, axis=0)

    kp2.append(kp[0])
    # kp2 = np.append(kp2, kp, axis=0)

# Draw founded keypoints
img1 = cv2.drawKeypoints(img1, kp1, img1)
img2 = cv2.drawKeypoints(img2, kp2, img2)
print(len(kp1))
print(len(kp2))

# Combine descriptors and kp
kp_all = []
des_all = []
kp_all.extend(kp1)
kp_all.extend(kp2)
des_all = np.append(des1, des2, axis=0)
print(len(kp_all))
print(des_all.shape)


#Store keypoint features
temp_array = []
temp = pickle_keypoints(kp_all, des_all)
temp_array.append(temp)
pickle.dump(temp_array, open("keypoints_database2.p", "wb"))

# Retrieve Keypoint Features
keypoints_database = pickle.load(open("keypoints_database2.p", "rb"))
kp_all_load, des_all_load = unpickle_keypoints(keypoints_database[0])


# # Draw loaded keypoints
img3 = cv2.drawKeypoints(img1, kp_all_load[0:99], img1)
img4 = cv2.drawKeypoints(img2, kp_all_load[100:199], img2)

# Show saved keypoints and loaded keypoints
plt.figure(),plt.imshow(img1),plt.title('saved1')
plt.figure(),plt.imshow(img2),plt.title('saved2')
plt.figure(),plt.imshow(img3),plt.title('loaded1')
plt.figure(),plt.imshow(img4),plt.title('loaded2')
plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows