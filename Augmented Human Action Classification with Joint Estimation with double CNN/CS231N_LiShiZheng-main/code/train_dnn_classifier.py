import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
# from scipy.io import savemat
from os.path import join
import matplotlib.pyplot as plt
# from classifier_baseline_cnn import * 
# from kaggel_dataload import *
from classifier_nn import *
from preprocess import *
dtype = torch.float32 # We will be using float throughout this tutorial.

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def main(val):
   
    optimizer = optim.SGD(model.parameters(),lr=0.001,
                        momentum=0.9, nesterov=True)
    
    data_dir = '../data/data-mpii' # TODO: Download and check
    loader_train, loader_val= load_data_and_preprocess_MPII(data_dir, args.batch_size)

    batch_size = 80
    num_batchs = int(train_images.shape[0]/batch_size)
    train_loss = []
    val_loss = []
    epoch = []
    val_epoch = []

    feature_extractor = feature_extractor()
    model = nn_Classifier(feature_extractor)
    model = model.to(device)
    model.train()
    for e in range(1):
        for i in range(num_batchs):
            model.train()  # put model to training mode
            x = train_images[i*batch_size:i*batch_size+batch_size]
            y = train_label[i*batch_size:i*batch_size+batch_size]
            
            x = x.to(device=device, dtype=dtype)  # move to device, e.g. GPU
            y = y.to(device=device, dtype=dtype)
            joints_cur = joints_cur.to(device=device, dtype=dtype)

            scores = model(x, joints_cur)
            
            loss = F.cross_entropy(scores, y) # TODO: Change this according to objective.
            

            # Zero out all of the gradients for the variables which the optimizer
            # will update.
            optimizer.zero_grad()

            # This is the backwards pass: compute the gradient of the loss with
            # respect to each  parameter of the model.
            loss.backward()

            # Actually update the parameters of the model using the gradients
            # computed by the backwards pass.
            optimizer.step()

            
            print('Iteration %d, loss = %.4f' % (i*batch_size+e*num_batchs*batch_size, loss.item()))
            train_loss.append(loss.item())
            cur_epoch = i*batch_size+e*num_batchs*batch_size
            epoch.append(cur_epoch)
            if val:
                if cur_epoch % (5*batch_size) == 0:
                    val_loss_cur = validate(model, val_images, val_label, val_joints, batch_size)
                    val_loss.append(val_loss_cur)
                    val_epoch.append(cur_epoch)
    plt.plot(epoch, train_loss)
    plt.ylabel('Training Loss')
    plt.xlabel('Epochs')
    plt.savefig("../outputs/classifer_training_loss.png")
    plt.close()
    if val:
        plt.plot(val_epoch, val_loss)
        plt.ylabel('Loss')
        plt.xlabel('Validation Epochs')
        plt.savefig("../outputs/classifer_validation_loss.png")
  

def validate(model, val_images, val_label, val_joints, batch_size):
    loss = 0.0
        # N = val_images.shape[0]
    num_batchs = int(val_images.shape[0]/batch_size)
    for i in range(num_batchs):
        model.train()  # put model to training mode
        x = val_images[i*batch_size:i*batch_size+batch_size]
        y = val_label[i*batch_size:i*batch_size+batch_size]
        joints_cur = val_joints[i*batch_size:i*batch_size+batch_size]
        x = x.to(device=device, dtype=dtype)  # move to device, e.g. GPU
        y = y.to(device=device, dtype=dtype)
        joints_cur = joints_cur.to(device=device, dtype=dtype)
        scores = model(x,joints_cur)
        temp = F.cross_entropy(scores, y) 
        loss += temp.item()
        # print("Validation Loss %.2f" %(temp))
    loss /= num_batchs
    # print("Final Validation Loss %.2f" %(loss))
    return loss


if __name__ == "__main__":
    # import configargparse

    # p = configargparse.ArgParser()


    # args = p.parse_args()
    val = True
    main(val)