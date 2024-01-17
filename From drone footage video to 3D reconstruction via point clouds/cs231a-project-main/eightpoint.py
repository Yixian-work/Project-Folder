import numpy as np
from skimage.io import imread
import matplotlib.pyplot as plt
import scipy.io as sio
from epipolar_utils import *

def lls_eight_point_alg(points1, points2):
    u = np.reshape(points2[:,0],(-1,1))
    v = np.reshape(points2[:,1],(-1,1))
    up = np.reshape(points1[:,0],(-1,1))
    vp = np.reshape(points1[:,1],(-1,1))
    W = np.hstack((u*up,u*vp,u,v*up,v*vp,v,up,vp,np.ones((np.size(u),1))))
    u,s,v_T = np.linalg.svd(W)
    f = v_T.T[:,-1]
    F_tilde = np.reshape(f,(3,3))
    uF,sF,v_TF = np.linalg.svd(F_tilde)
    sF[2] = 0.
    F = np.matmul(uF, np.matmul(np.diag(sF), v_TF))
    return F

def normalized_eight_point_alg(points1, points2):
    u = np.reshape(points2[:,0],(-1,1))
    v = np.reshape(points2[:,1],(-1,1))
    up = np.reshape(points1[:,0],(-1,1))
    vp = np.reshape(points1[:,1],(-1,1))
    uc = np.mean(u)
    vc = np.mean(v)
    upc = np.mean(up)
    vpc = np.mean(vp)
    cpt = np.array([[uc, vc, 1.]])
    cptp = np.array([[upc, vpc, 1.]])
    Tr = np.array([[1., 0., -uc],
                   [0., 1., -vc],
                   [0., 0., 1.]])
    Trp = np.array([[1., 0., -upc],
                    [0., 1., -vpc],
                    [0., 0., 1.]])
    s = np.sqrt(2*np.size(u)/(np.sum(np.square(points2-cpt))))
    sp = np.sqrt(2*np.size(up)/(np.sum(np.square(points1-cptp))))
    R = np.array([[s, 0., 0.],
                  [0., s, 0.],
                  [0., 0., 1.]])
    Rp = np.array([[sp, 0., 0.],
                   [0., sp, 0.],
                   [0., 0., 1. ]])
    T = np.matmul(R, Tr)
    Tp = np.matmul(Rp, Trp)
    q = np.matmul(T, np.hstack((u, v, np.ones((np.size(u),1)))).T).T
    qp = np.matmul(Tp, np.hstack((up, vp, np.ones((np.size(up),1)))).T).T
    # eight point algorithm
    Fq = lls_eight_point_alg(qp[:,:2], q[:,:2])
    # Return back to original
    F = np.matmul(T.T, np.matmul(Fq, Tp))
    return F

def plot_epipolar_lines_on_images(points1, points2, im1, im2, F):

    def plot_epipolar_lines_on_image(points1, points2, im, F):
        im_height = im.shape[0]
        im_width = im.shape[1]
        lines = F.T.dot(points2.T)
        plt.imshow(im, cmap='gray')
        for line in lines.T:
            a,b,c = line
            xs = [1, im.shape[1]-1]
            ys = [(-c-a*x)/b for x in xs]
            plt.plot(xs, ys, 'r')
        for i in range(points1.shape[0]):
            x,y,_ = points1[i]
            plt.plot(x, y, '*b')
        plt.axis([0, im_width, im_height, 0])

    new_figsize = (8 * (float(max(im1.shape[1], im2.shape[1])) / min(im1.shape[1], im2.shape[1]))**2 , 6)
    fig = plt.figure(figsize=new_figsize)
    plt.subplot(121)
    plot_epipolar_lines_on_image(points1, points2, im1, F)
    plt.axis('off')
    plt.subplot(122)
    plot_epipolar_lines_on_image(points2, points1, im2, F.T)
    plt.axis('off')


def compute_distance_to_epipolar_lines(points1, points2, F):
    lines = np.matmul(F.T, points2.T)
    l = (lines/np.linalg.norm(lines[:2,:],axis=0)).T
    distances = np.abs(l[:,0]*points1[:,0]+l[:,1]*points1[:,1]+l[:,2])/np.sqrt(l[:,0]**2+l[:,1]**2)
    average_distance = np.average(distances)
    return average_distance
