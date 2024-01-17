import torch
import torchvision.datasets as dset
import torchvision.transforms as T
from torch.utils.data import DataLoader
from torch.utils.data import sampler
import numpy as np
from scipy.io import loadmat
from PIL import Image
import glob
import cv2

def write_MPII(data_dir, batch_size=64):

    # Data import
    mat = loadmat(data_dir + "/mpii_test")
    cat_name = mat['cat_name']
    pic_name = mat['pic_name']
    joint_loc = mat['joint_loc']
    label = mat['label'].reshape(-1)

    # Data processing
    datlen = np.size(label)
    # datlen = 10000

    # Print out messages related to format
    print(joint_loc[0][1][0][3][2][0][0]) # Second index: N, Fourth: joint, Fifth: x, y, and joint id
    print(pic_name[0][1][0])
    print(np.size(joint_loc[0][1][0]))

    # Arrange datas
    joint_info = np.zeros((datlen, 32, 2)) # 32 = 16 * 2
    input_info = np.zeros((datlen, 3, 224, 224))
    error_list = []
    invalid = {'040348287.jpg', '013401523.jpg','002878268.jpg'}
    # Load pictures, stack datas
    for N in range(datlen):
        num_vis = np.size(joint_loc[0][N][0])
        pic = pic_name[0][N][0]
        if pic in invalid:
            error_list.append(1)
            continue
        img = Image.open(data_dir+"/images/" + pic) # TODO: Make sure you are naming the image folder correctly
        height, width = np.array(img).shape[0], np.array(img).shape[1] # Original height and width
        img_resize = img.resize((224, 224)) # Transform to 224 * 224 image
        trans_ratio_h = 224.0 / height
        trans_ratio_w = 224.0 / width
        # trans_ratio = (trans_ratio_h, trans_ratio_w) # Height width ratio format
        x_raw = np.array(img_resize)
        x = x_raw - np.mean(x_raw, axis=(1,2), keepdims=True) # Per-channel, per-image zero-centering
        x = x.transpose(2, 0, 1)
        input_info[N,:,:,:] = x
        # invisible points are marked with 0 invisibility. Visible are marked with 1 invisibility.
        error = 0
        for j in range(num_vis):
            id = joint_loc[0][N][0][j][2][0][0]
            if id > 15:
                error = 1
                error_list.append(error)
                break
        if error == 0:
            error_list.append(error)
            for j in range(num_vis):
                id = joint_loc[0][N][0][j][2][0][0]         
                joint_info[N, int(2*id), 0] = joint_loc[0][N][0][j][0][0][0] * trans_ratio_w - 112 # NOTICE! Subtract a value here to "center"
                joint_info[N, int(2*id+1), 0] = joint_loc[0][N][0][j][1][0][0] * trans_ratio_h - 112
                joint_info[N, int(2*id), 1] = 1 #visibality
                joint_info[N, int(2*id+1), 1] = 1
    
    print(np.sum(error_list))
    error_list_np = np.array(error_list)
    # Store everything in a file.
    joint_info_reshaped = joint_info.reshape(joint_info.shape[0],-1)
    # np.savetxt(data_dir + "/joint_info.csv", joint_info_reshaped, delimiter=',')
    joint_info_reshaped.tofile(data_dir + "/joint_info.csv", sep=',')
    print('joint info done.')
    input_info_reshaped = input_info.reshape(input_info.shape[0],-1)
    # np.savetxt(data_dir + "/input_info.csv", input_info_reshaped, delimiter=',')
    # np.savetxt(data_dir + "/error_list.csv", error_list_np, delimiter=',')
    # np.savetxt(data_dir + "/label.csv", label, delimiter=",")
    input_info_reshaped.tofile(data_dir + "/input_info.csv", sep=',')
    print('input info done.')
    error_list_np.tofile(data_dir + "/error_list.csv", sep=',')
    print('error list done.')
    label.tofile(data_dir + "/label.csv", sep=',')
    print('label done.')


if __name__ == "__main__":
    write_MPII("../data/data-mpii")
