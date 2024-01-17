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
import random
# Specific loading and preprocession of leeds dataset
def load_data_and_preprocess_leeds(data_dir, batch_size):

    NUM_TRAIN = 10000 # TODO: num of train samples we want
    NUM_VAL = 1000 # TODO: num of validation samples we want

    # Label processing
    mat = loadmat(data_dir + "/joints")
    joints_labels = mat['joints'] # in numpy form 14 * 3 * N
    joints_labels_trans = np.transpose(joints_labels, (2, 0, 1)) # N * 14 * 3
    # print(joints_labels_trans.shape)
    joints_locs = joints_labels_trans[:,:,:2] # N * 14 * 2
    joints_locs[:,:,[0, 1]] = joints_locs[:,:,[1, 0]] # Switch width and height index so it is consistent with operation below.
    # print(joints_locs.shape)
    joints_delocs = joints_locs.reshape(-1, 28, 1) # N * 28 * 1

    # Flag manipulation
    joints_flags = joints_labels_trans[:,:,2].reshape(-1, 14, 1) # N * 14 * 1
    joints_flags_tile = np.tile(joints_flags, (1, 1, 2)) # N * 14 * 2
    joints_deflags = joints_flags_tile.reshape(-1, 28, 1) # N * 28 * 1

    # formulate new labels
    labels = np.concatenate((joints_delocs, joints_deflags), axis=2) # N * 28 * 2

    # Data processing
    trainloader = []
    valloader = []
    testloader = []

    filelist = glob.glob(data_dir + "/images/*.jpg") # TODO: Modify the path
    N = 0
    for fname in filelist:
        img = Image.open(fname)
        height, width = np.array(img).shape[0], np.array(img).shape[1] # Original height and width
        img_resize = img.resize((96, 96)) # Transform to 96 * 96 image
        trans_ratio_h = 96.0 / height
        trans_ratio_w = 96.0 / width
        trans_ratio = (trans_ratio_h, trans_ratio_w) # Height width ratio format
        x_raw = np.array(img_resize)
        x = x_raw - np.mean(x_raw, axis=(1,2), keepdims=True) # Per-channel, per-image zero-centering
        x = x.transpose(2, 0, 1)
        # Reflect this change on y label as well
        y = labels[N,:,:] # Extract per-sample 28 * 2
        trans_ratio_reshape = (np.tile(np.array(trans_ratio), (14, 1))).reshape(-1)
        y[:,0] *= trans_ratio_reshape # Make transform on loc data of y
        # Segment the dataset into train, val, and test based on a ratio of 8:1:1
        t = N
        if N % 10 == 3:
            valloader.append((t, torch.from_numpy(x), torch.from_numpy(y), trans_ratio_h, trans_ratio_w))
        elif N % 10 == 6:
            testloader.append((t, torch.from_numpy(x), torch.from_numpy(y), trans_ratio_h, trans_ratio_w))
        else:
            trainloader.append((t, torch.from_numpy(x), torch.from_numpy(y), trans_ratio_h, trans_ratio_w))
        N += 1

    trainloader, valloader, testloader = DataLoader(trainloader, batch_size=batch_size), DataLoader(valloader, batch_size=batch_size), DataLoader(testloader, batch_size=batch_size)
    return trainloader, valloader, testloader

def load_data_and_preprocess_MPII(labels_check, data_dir, batch_size=64):

    # Data import
    mat = loadmat(data_dir + "/mpii_test")
    cat_name = mat['cat_name']
    pic_name = mat['pic_name']
    joint_loc = mat['joint_loc']
    label = mat['label'].reshape(-1)
    # c = list(zip(cat_name, pic_name, joint_loc, label))

    # random.shuffle(c)

    # cat_name, pic_name, joint_loc, label = zip(*c)

    # Data processing
    datlen = np.size(label)
    # datlen = 2000

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
        x = x_raw - np.mean(x_raw, axis=(0,1), keepdims=True) # Per-channel, per-image zero-centering
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

    
    # Up to this point we have input_info (N, 3, 224, 224), joint_info (N, 32, 2), and label (N,), we group them as 
    # training set and validation set. We also perform tensor transformation.
    train_list = []
    val_list = [] 
    max_l = 0
    for N in range(datlen):
        cur_label = label[N]
        if label[N] == 6 or label[N] == 11 or label[N] == 13 or label[N] == 15 or label[N] == 16:
            # print("minority")
            continue
        if cur_label > 6 and cur_label < 11:
            cur_label-=1
        elif label[N] == 12:
            cur_label-=2
        elif cur_label== 14:
            cur_label-=3
        elif cur_label > 16:
            cur_label-=5
        labels_check[cur_label] += 1
        if error_list[N] == 1:
            continue
        decider = np.random.randint(10)
        if cur_label > max_l:
            max_l = cur_label
        if decider == 0:
            val_list.append((torch.from_numpy(input_info[N,:,:,:]), torch.from_numpy(joint_info[N,:,:]), cur_label))
        else:
            train_list.append((torch.from_numpy(input_info[N,:,:,:]), torch.from_numpy(joint_info[N,:,:]), cur_label))
    random.shuffle(train_list)
    trainloader, valloader = DataLoader(train_list, batch_size=batch_size), DataLoader(val_list, batch_size=batch_size)
    return trainloader, valloader

def load_data_and_preprocess_MPII_csv(data_dir, batch_size):
        # Load data from data_dir path
    joint_info_reshape = np.loadtxt(data_dir + "/joint_info.csv", delimiter=",")
    joint_info = joint_info_reshape.reshape(joint_info_reshape.shape[0]//64, 32, 2)
    input_info_reshape = np.loadtxt(data_dir + "/input_info.csv", delimiter=",")
    input_info = input_info_reshape.reshape(input_info_reshape.shape[0]//(3*224*224), 3, 224, 224)
    label = np.loadtxt(data_dir + "/label.csv", delimiter=",")
    error_list = np.loadtxt(data_dir + "/error_list.csv", delimiter=",")
    print('read complete.')
    datlen = 10000
    # Up to this point we have input_info (N, 3, 224, 224), joint_info (N, 32, 2), and label (N,), we group them as 
    # training set and validation set. We also perform tensor transformation.
    train_list = []
    val_list = [] 
    for N in range(datlen):
        if (N % 1000) == 0:
            print('.')
        if error_list[N] == 1:
            continue
        # decider = np.random.randint(10)
        decider = N % 8
        if decider == 0:
            val_list.append((torch.from_numpy(input_info[N,:,:,:]), torch.from_numpy(joint_info[N,:,:]), label[N]))
        else:
            train_list.append((torch.from_numpy(input_info[N,:,:,:]), torch.from_numpy(joint_info[N,:,:]), label[N]))
    trainloader, valloader = DataLoader(train_list, batch_size=batch_size), DataLoader(val_list, batch_size=batch_size)
    return trainloader, valloader