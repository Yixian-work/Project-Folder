import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matching import *
from eightpoint import *
from triangulation import *
from ransac import *

def pipeline():
    return

if __name__ == '__main__':
    path1 = r'.\raw_pictures\frame650.jpg'
    path2 = r'.\raw_pictures\frame800.jpg'
    path3 = r'.\raw_pictures\frame400.jpg'
    kp1, des1 = keySIFT(path1)
    kp2, des2 = keySIFT(path2)
    list1, list2 = matching(kp1,des1,kp2,des2)
    best_F, inlier, numInlier = ransac(list1, list2)
    selected1 = [list1[inl] for inl in inlier]
    ss1 = np.asarray(selected1)
    selected2 = [list2[inl] for inl in inlier]
    ss2 = np.asarray(selected2)
    ss = np.concatenate((ss1, ss2), axis=1)
    F = normalized_eight_point_alg(np.concatenate((ss1, np.ones((numInlier, 1))), axis=1),np.concatenate((ss2, np.ones((numInlier, 1))), axis=1))
    e11 = compute_epipole(np.concatenate((ss1, np.ones((numInlier, 1))), axis=1),np.concatenate((ss2, np.ones((numInlier, 1))), axis=1), F)
    # e2 = compute_epipole(np.concatenate((ss2, np.ones((numInlier, 1))), axis=1),np.concatenate((ss1, np.ones((numInlier, 1))), axis=1), F.T)
    P1, P2 = findProjMatrix(F, e11)
    camera_matrices = np.reshape(np.concatenate((P1, P2), axis=0), (-1,3,4))
    Pps = np.zeros((numInlier, 3))
    for i in range(numInlier):
        image_points = np.reshape(ss[i,:], (2, 2))
        Pp = linear_estimate_3d_point(image_points, camera_matrices)
        Pps[i,:] = Pp
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.scatter(Pps[:,0], Pps[:,1], Pps[:,2], c='k', depthshade=True, s=2)
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 4000)
    ax.set_zlim(-5, 5)
    plt.show()

    kp3, des3 = keySIFT(path3)
    list1_new, list2_new = matching(kp1,des1,kp3,des3)
