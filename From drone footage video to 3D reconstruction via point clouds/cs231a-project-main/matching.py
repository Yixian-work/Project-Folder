# This file is util of preprocessing the images.
import numpy as np
import matplotlib.pyplot as plt
from skimage import io
import os
import cv2 as cv

# This part of code is adapted from cv doc. Experiment: will not be used.
def keymatchesORB(path1, path2):
    img1 = cv.imread(path1, 0)
    img2 = cv.imread(path2, 0)
    orb = cv.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)
    matches = sorted(matches, key = lambda x:x.distance)
    img3 = cv.drawMatches(img1, kp1, img2, kp2, matches, None, flags=2)
    cv.imwrite(r'.\processed_pictures\sift_keypoints1.jpg',img3)

# This part of code is adapted from cv doc. We will use this! This returns two
# lists of tuples (x, y) that corresponds to pairs of corresponding points.
def keySIFT(path):
    img = cv.imread(path, 0)
    sift = cv.SIFT_create()
    kp, des = sift.detectAndCompute(img, None)
    img = cv.drawKeypoints(img, kp, outImage = None, color=(255,0,0))
    cv.imwrite(r'.\processed_pictures\sift_features.jpg', img)
    return kp, des

def matching(kp1, des1, kp2, des2):
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    list_1 = []
    list_2 = []
    good = []
    for m,n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    for match in good:
        img1_idx = match.queryIdx
        img2_idx = match.trainIdx
        (x1, y1) = kp1[img1_idx].pt
        (x2, y2) = kp2[img2_idx].pt
        list_1.append((x1, y1))
        list_2.append((x2, y2))
    return list_1, list_2

def keymatchesSIFT(path1, path2):
    img1 = cv.imread(path1, 0)
    img2 = cv.imread(path2, 0)
    sift = cv.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    bf = cv.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)
    list_1 = []
    list_2 = []
    good = []
    for m,n in matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)
    for match in good:
        img1_idx = match.queryIdx
        img2_idx = match.trainIdx
        (x1, y1) = kp1[img1_idx].pt
        (x2, y2) = kp2[img2_idx].pt
        list_1.append((x1, y1))
        list_2.append((x2, y2))
    print(np.shape(good))
    img3 = cv.drawMatches(img1, kp1, img2, kp2, good, None, flags=2)
    cv.imwrite(r'.\processed_pictures\sift_keypoints3.jpg',img3)
    return list_1, list_2

# def corner():
#     img = cv.imread(r".\raw_pictures\frame850.jpg")
#     gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
#     corners = cv.goodFeaturesToTrack(gray,1500,0.01,10)
#     corners = np.int0(corners)
#     for i in corners:
#         x,y = i.ravel()
#         cv.circle(img,(x,y),3,255,-1)
#     plt.imshow(img),plt.show()
#     return

# This part of code is adapted from cv doc. Experiment: Will not be used.
# def keymatchesSIFT1(path1, path2):
#     img1 = cv.imread(path1, 0)
#     img2 = cv.imread(path2, 0)
#     sift = cv.SIFT_create()
#     kp1, des1 = sift.detectAndCompute(img1, None)
#     kp2, des2 = sift.detectAndCompute(img2, None)
#     bf = cv.BFMatcher(cv.NORM_L1, crossCheck=True)
#     matches = bf.match(des1, des2)
#     matches = sorted(matches, key = lambda x:x.distance)
#     img3 = cv.drawMatches(img1,kp1,img2,kp2,matches[:200],img2,flags=2)
#     cv.imwrite(r'.\processed_pictures\sift_keypoints2.jpg',img3)

# def fast():
#     img = cv.imread(r".\raw_pictures\frame850.jpg",0)
#     fast = cv.FastFeatureDetector()
#     kp = fast.detect(img,None)
#     img2 = cv.drawKeypoints(img, kp, color=(255,0,0))
#     cv.imwrite(r'.\processed_pictures\fast_keypoints1.jpg',img2)
#     fast.setBool('nonmaxSuppression',0)
#     kp = fast.detect(img,None)
#     img3 = cv.drawKeypoints(img, kp, color=(255,0,0))
#     cv.imwrite(r'.\processed_pictures\fast_keypoints2.jpg',img3)
#     return

# Frame get function. Stored in target directory. ALready been used.
def capture():
    videoFile = r".\raw_pictures\production.mp4"
    vidcap = cv.VideoCapture(videoFile)
    success,image = vidcap.read()
    seconds = 1
    fps = vidcap.get(cv.CAP_PROP_FPS) # Gets the frames per second
    multiplier = fps * seconds
    while success:
        frameId = int(round(vidcap.get(1)))
        success, image = vidcap.read()
        if frameId % multiplier == 0:
            cv.imwrite(r".\raw_pictures\frame%d.jpg" % frameId, image)
    vidcap.release()
    print("Complete")

if __name__ == '__main__':
    path = r'.\raw_pictures\frame800.jpg'
    keySIFT(path)
