import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matching import *
from eightpoint import *
from triangulation import *
from ransac import *

def pipeline():
    return

if __name__ == '__main__':
    # Specify paths
    path1 = r'.\raw_pictures\frame650.jpg'
    path2 = r'.\raw_pictures\frame800.jpg'
    path3 = r'.\raw_pictures\frame400.jpg'
    kp1, des1 = keySIFT(path1)
    kp2, des2 = keySIFT(path2)
    list1, list2 = matching(kp1,des1,kp2,des2)
    pts1 = np.int32(list1) - np.array([[960, 540]]) # Center the principal point
    pts2 = np.int32(list2) - np.array([[960, 540]]) # Center the principal point
    # best_F, inlier, numInlier = ransac(list1, list2)
    # selected1 = [list1[inl] for inl in inlier]
    # pts1 = np.asarray(selected1)
    # selected2 = [list2[inl] for inl in inlier]
    # pts2 = np.asarray(selected2)
    # F = normalized_eight_point_alg(np.concatenate((pts1, np.ones((numInlier, 1))), axis=1),np.concatenate((pts2, np.ones((numInlier, 1))), axis=1))
    F, mask = cv.findFundamentalMat(pts1,pts2,cv.FM_RANSAC)
    pts1 = pts1[mask.ravel()==1]
    pts2 = pts2[mask.ravel()==1]
    e11 = compute_epipole(np.concatenate((pts1, np.ones((np.shape(pts1)[0], 1))), axis=1),np.concatenate((pts2, np.ones((np.shape(pts1)[0], 1))), axis=1), F)
    P1, P2 = findProjMatrix(F, e11)
    camera_matrices = np.reshape(np.vstack((P1, P2)), (-1,3,4))
    points_3d_list = []
    for i in range(np.shape(pts1)[0]):
        pt1 = np.reshape(pts1[i,:],(1,2))
        pt2 = np.reshape(pts2[i,:],(1,2))
        image_point = np.vstack((pt1,pt2))
        Pp = linear_estimate_3d_point(image_point, camera_matrices)
        points_3d_list.append(Pp)
    points_3d = np.asarray(points_3d_list)
    # ss = np.concatenate((ss1, ss2), axis=1)
    # F = normalized_eight_point_alg(np.concatenate((ss1, np.ones((numInlier, 1))), axis=1),np.concatenate((ss2, np.ones((numInlier, 1))), axis=1))
    # e11 = compute_epipole(np.concatenate((ss1, np.ones((numInlier, 1))), axis=1),np.concatenate((ss2, np.ones((numInlier, 1))), axis=1), F)
    # P1, P2 = findProjMatrix(F, e11)
    # camera_matrices = np.reshape(np.concatenate((P1, P2), axis=0), (-1,3,4))
    # Pps = np.zeros((numInlier, 3))
    # for i in range(numInlier):
    #     image_points = np.reshape(ss[i,:], (2, 2))
    #     Pp = linear_estimate_3d_point(image_points, camera_matrices)
    #     Pps[i,:] = Pp
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(points_3d[:,0], points_3d[:,1], points_3d[:,2], c='k', depthshade=True, s=2)
    ax.set_xlim(-2000, 2000)
    ax.set_ylim(-2000, 2000)
    ax.set_zlim(-50, 50)
    plt.show()
    # Add a view.
    kp3, des3 = keySIFT(path3)
    list1_new, list3 = matching(kp1,des1,kp3,des3)
    pts1_new = np.int32(list1_new) - np.array([[960, 540]])
    pts3 = np.int32(list3) - np.array([[960, 540]])
    # best_F, inlier, numInlier = ransac(list1_new, list3)
    # selected1 = [list1_new[inl] for inl in inlier]
    # pts1_new = np.asarray(selected1)
    # selected2 = [list3[inl] for inl in inlier]
    # pts3 = np.asarray(selected2)
    # F = normalized_eight_point_alg(np.concatenate((pts1_new, np.ones((numInlier, 1))), axis=1),np.concatenate((pts3, np.ones((numInlier, 1))), axis=1))
    F_new, mask_new = cv.findFundamentalMat(pts1_new,pts3,cv.FM_RANSAC)
    pts1_new = pts1_new[mask_new.ravel()==1]
    pts3 = pts3[mask_new.ravel()==1]
    e12 = compute_epipole(np.concatenate((pts1_new, np.ones((np.shape(pts1_new)[0], 1))), axis=1),np.concatenate((pts3, np.ones((np.shape(pts1_new)[0], 1))), axis=1), F)
    P1_new, P3 = findProjMatrix(F, e12)
    camera_matrices = np.reshape(np.vstack((P1_new, P3)), (-1,3,4))
    existed_points = []
    new_points = []
    # categorize the points into existed points and new points seperately.
    for i in range(np.shape(pts1_new)[0]):
        searchKey = pts1_new[i,:].reshape(1,2)
        if np.any(np.sum(np.abs(pts1-searchKey), axis=1) == 0):
            existed_points.append(pts1_new[i,:])
        else:
            new_points.append(pts1_new[i,:])
    # linear estimate points
    for i in range(np.shape(pts1_new)[0]):
        pt1 = np.reshape(pts1_new[i,:],(1,2))
        pt2 = np.reshape(pts3[i,:],(1,2))
        image_point = np.vstack((pt1,pt2))
        Pp = linear_estimate_3d_point(image_point, camera_matrices)
        points_3d_list.append(Pp)
    points_3d = np.asarray(points_3d_list)
    # points_3d_add = []
    # for i in range(np.shape(pts1)[0]):
    #     pt1 = np.reshape(pts1[i,:],(1,2))
    #     pt2 = np.reshape(pts2[i,:],(1,2))
    #     image_point = np.vstack((pt1,pt2))
    #     Pp = linear_estimate_3d_point(image_point, camera_matrices)
    #     points_3d_add.append(Pp)
    # points_3d_add = np.asarray(points_3d_add)
    H = findH(P1,P2,P3)
    points_3d_h = np.hstack((points_3d, np.ones((np.shape(points_3d)[0], 1))))
    new_points_3d_h = np.linalg.inv(H).dot(points_3d_h.T).T
    new_points_3d = new_points_3d_h[:,:3]/new_points_3d_h[:,3].reshape(-1,1)
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(new_points_3d[:,0], new_points_3d[:,1], new_points_3d[:,2], c='k', depthshade=True, s=2)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)
    plt.show()
