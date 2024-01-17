import numpy as np

def cross_product_mat(a):
    m = np.zeros((3,3))
    m[1,0] = a[2]
    m[0,1] = -a[2]
    m[2,0] = -a[1]
    m[0,2] = a[1]
    m[2,1] = a[0]
    m[1,2] = -a[0]
    return m

# def findEpipole1(F):
#     u, s, vT = np.linalg.svd(F.T)
#     v = vT.T[:,2]
#     e = v/np.linalg.norm(v)
#     return e

def compute_epipole(points1, points2, F):
    # TODO: Implement this method!
    L = np.matmul(F.T, points2.T).T
    u,s,vT = np.linalg.svd(L)
    e_raw = vT.T[:,-1]
    epipole = e_raw/e_raw[2]
    return epipole

def findProjMatrix(F, e):
    M1 = np.concatenate((np.identity(3), np.zeros((3,1))), axis=1)
    A = np.matmul(cross_product_mat(e), F)
    b = np.reshape(e, (3,1))
    M2 = np.concatenate((A, b), axis=1)
    return M1, M2

def linear_estimate_3d_point(image_points, camera_matrices):
    M3 = camera_matrices[:,2,:]
    M1 = camera_matrices[:,0,:]
    M2 = camera_matrices[:,1,:]
    p1 = np.reshape(image_points[:,0],(-1,1))
    p2 = np.reshape(image_points[:,1],(-1,1))
    A1 = p1*M3-M1
    A2 = p2*M3-M2
    A = np.reshape(np.hstack((A1,A2)),(-1,4))
    u,s,vT = np.linalg.svd(A)
    point_3d_raw = vT.T[:,3]
    point_3d = (point_3d_raw/(point_3d_raw[3]))[:3]
    return point_3d

def reprojection_error(point_3d, image_points, camera_matrices):
    X = np.concatenate((point_3d, [1]))
    y = np.dot(camera_matrices, X)
    pp = y[:,0:2]/np.reshape(y[:,2],(-1,1))
    error = np.reshape(pp-image_points,(-1,))
    return error

def jacobian(point_3d, camera_matrices):
    X = np.concatenate((point_3d, [1]))
    M3 = camera_matrices[:,2,:]
    M1 = camera_matrices[:,0,:]
    M2 = camera_matrices[:,1,:]
    v = np.dot(M3, X).reshape((-1,1))
    u1 = np.dot(M1, X).reshape((-1,1))
    u2 = np.dot(M2, X).reshape((-1,1))
    jacobian1 =  (v*M1[:,:3]-u1*M3[:,:3])/(v**2)
    jacobian2 =  (v*M2[:,:3]-u2*M3[:,:3])/(v**2)
    jacobian = np.reshape(np.hstack((jacobian1, jacobian2)),(-1,3))
    return jacobian

def nonlinear_estimate_3d_point(image_points, camera_matrices):
    # initialize Pp using an approximation of linear estimate and set iterations
    Pp = linear_estimate_3d_point(image_points, camera_matrices)
    iterations = 30
    # Gauss-Newton algorithm
    for k in range(iterations):
        J = jacobian(Pp, camera_matrices)
        e = reprojection_error(Pp, image_points, camera_matrices)
        Pp = Pp - np.dot(np.matmul(np.linalg.inv(np.matmul(J.T,J)), J.T),e)
    # get output
    return Pp

def computeFourConstraints(P): # From Pollefey's paper
    coeff1 = np.array([P[0,0]**2-P[1,0]**2, 2*(P[0,0]*P[0,1]-P[1,0]*P[1,1]), 2*(P[0,0]*P[0,2]-P[1,0]*P[1,2]), 2*(P[0,0]*P[0,3]-P[1,0]*P[1,3]),
                       P[0,1]**2-P[1,1]**2, 2*(P[0,1]*P[0,2]-P[1,1]*P[1,2]), 2*(P[0,1]*P[0,3]-P[1,1]*P[1,3]),
                       P[0,2]**2-P[1,2]**2, 2*(P[0,2]*P[0,3]-P[1,2]*P[1,3]),
                       P[0,3]**2-P[1,3]**2]).reshape(1,10)
    coeff2 = np.array([P[1,0]*P[0,0], P[1,0]*P[0,1]+P[1,1]*P[0,0], P[1,0]*P[0,2]+P[1,2]*P[0,0], P[1,0]*P[0,3]+P[1,3]*P[0,0],
                       P[1,1]*P[0,1], P[1,1]*P[0,2]+P[1,2]*P[0,1], P[1,1]*P[0,3]+P[1,3]*P[0,1],
                       P[1,2]*P[0,2], P[1,2]*P[0,3]+P[1,3]*P[0,2],
                       P[1,3]*P[0,3]]).reshape(1,10)
    coeff3 = np.array([P[2,0]*P[0,0], P[2,0]*P[0,1]+P[2,1]*P[0,0], P[2,0]*P[0,2]+P[2,2]*P[0,0], P[2,0]*P[0,3]+P[2,3]*P[0,0],
                       P[2,1]*P[0,1], P[2,1]*P[0,2]+P[2,2]*P[0,1], P[2,1]*P[0,3]+P[2,3]*P[0,1],
                       P[2,2]*P[0,2], P[2,2]*P[0,3]+P[2,3]*P[0,2],
                       P[2,3]*P[0,3]]).reshape(1,10)
    coeff4 = np.array([P[2,0]*P[1,0], P[2,0]*P[1,1]+P[2,1]*P[1,0], P[2,0]*P[1,2]+P[2,2]*P[1,0], P[2,0]*P[1,3]+P[2,3]*P[1,0],
                       P[2,1]*P[1,1], P[2,1]*P[1,2]+P[2,2]*P[1,1], P[2,1]*P[1,3]+P[2,3]*P[1,1],
                       P[2,2]*P[1,2], P[2,2]*P[1,3]+P[2,3]*P[1,2],
                       P[2,3]*P[1,3]]).reshape(1,10)
    coeff = np.vstack((coeff1, coeff2, coeff3, coeff4))
    return coeff

def findH(P1, P2, P3): # Three matrices case
    coeffMat = np.vstack((computeFourConstraints(P1), computeFourConstraints(P2), computeFourConstraints(P3)))
    u, s, vT = np.linalg.svd(coeffMat)
    c = vT.T[:,-1]
    Omega = np.array([[c[0], c[1], c[2], c[3]],
                      [c[1], c[4], c[5], c[6]],
                      [c[2], c[5], c[7], c[8]],
                      [c[3], c[6], c[8], c[9]]])
    Omega = Omega / Omega[3,3]
    A = np.linalg.cholesky(np.linalg.inv(Omega[:3,:3])).T
    K = np.linalg.inv(A)
    K = K/K[2,2]
    p = -np.linalg.inv(K.dot(K.T)).dot(Omega[:3,3]).reshape(1,3)
    H_1 = np.hstack((K, np.zeros((3,1))))
    H_2 = np.hstack((-p.dot(K), [[1]]))
    H = np.vstack((H_1, H_2))
    return H
