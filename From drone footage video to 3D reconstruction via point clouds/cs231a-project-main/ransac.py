import numpy as np
from eightpoint import *

minSamples = 8
thres = 0.05

# The ransac algorithm returns a fundamental matrix between two cameras that have
# the most inliers. It utilizes eight point algorithm.
def ransac(list_1, list_2):
    numSamples = len(list_1)
    best_F = None
    numInlier = 0
    best_inlier_idx = None
    for i in range(numSamples):
        randSamples = np.random.choice(numSamples, minSamples, replace=False)
        selectedSample_1 = [list_1[rand] for rand in randSamples]
        ss1 = np.concatenate((np.asarray(selectedSample_1), np.ones((minSamples, 1))), axis=1)
        selectedSample_2 = [list_2[rand] for rand in randSamples]
        ss2 = np.concatenate((np.asarray(selectedSample_2), np.ones((minSamples, 1))), axis=1)
        F = normalized_eight_point_alg(ss1, ss2)
        inlier = 0
        inlierIdx = np.zeros(numSamples)
        p1 = np.concatenate((list_1, np.ones((numSamples, 1))), axis=1)
        p2 = np.concatenate((list_2, np.ones((numSamples, 1))), axis=1)
        dist = np.diag(np.matmul(np.matmul(p2, F), p1.T))
        inlierIdx = np.where(np.abs(dist) < thres)[0]
        inlier = np.size(inlierIdx)
        if numInlier <= inlier:
            numInlier = inlier
            best_inlier_idx = inlierIdx
            best_F = F
    return best_F, best_inlier_idx, numInlier

def ransac2(list_1, list_2):
    numSamples = len(list_1)
    best_F = None
    numInlier = 0
    best_inlier_idx = None
    while 1:
        randSamples = np.random.choice(numSamples, minSamples, replace=False)
        selectedSample_1 = [list_1[rand] for rand in randSamples]
        ss1 = np.concatenate((np.asarray(selectedSample_1), np.ones((minSamples, 1))), axis=1)
        selectedSample_2 = [list_2[rand] for rand in randSamples]
        ss2 = np.concatenate((np.asarray(selectedSample_2), np.ones((minSamples, 1))), axis=1)
        F = normalized_eight_point_alg(ss1, ss2)
        inlier = 0
        inlierIdx = np.zeros(numSamples)
        p1 = np.concatenate((list_1, np.ones((numSamples, 1))), axis=1)
        p2 = np.concatenate((list_2, np.ones((numSamples, 1))), axis=1)
        dist = np.diag(np.matmul(np.matmul(p2, F), p1.T))
        inlierIdx = np.where(np.abs(dist) < thres)[0]
        inlier = np.size(inlierIdx)
        if inlier/numSamples >= 0.95:
            best_F = F
            best_inlier_idx = inlierIdx
            numInlier = inlier
            break
    return best_F, best_inlier_idx, numInlier
