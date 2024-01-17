import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from scipy.io import savemat
from os.path import join
from draw_key_points import *
import util
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter

# writer.add_scalar("Loss/validation", totalloss/ len(valid_data_loader), (epoch-1)*270)
dtype = torch.float32 # We will be using float throughout this tutorial.

device = torch.device('cuda')

# Constant to control how frequently we print train loss.
# print_every = 100
writer = SummaryWriter()
def train_part(model, optimizer, loader_train, loader_val, args, lr_scheduler):
    """
    Inputs:
    - model: A PyTorch Module giving the model to train.
    - optimizer: An Optimizer object we will use to train the model
    - loader_train: loader_train (training dataset)
    - loader_val: loader_val (validation dataset)
    - args: hyperparameters

    Returns: Nothing, but prints model accuracies during training.
    """
    args.save_dir = util.get_save_dir(args.save_dir, args.name, training=True)

    model = model.to(device)
    model.train()
    saver = util.CheckpointSaver(args.save_dir,
                                 max_checkpoints=args.max_checkpoints,
                                 criteria=args.crit)
    batch_size = args.batch_size
    steps_to_eval = args.eval_steps
    total_steps = 0
    timestep = 0
    eval_timestep = 0

    # class_loss = nn.KLDivLoss(reduction='batchmean', log_target=True)
    class_loss = nn.CrossEntropyLoss()
    model2 = None
    if args.step == 'classification' and args.ground_truth == 0:
        model2 = DeepPose(16)
        model2 = util.load_model(model2, args.load_path)
        model2 = model2.to(device)
        model2.eval()

    for e in range(args.num_epochs):
        with torch.enable_grad(), \
                tqdm(total=len(loader_train.dataset)) as progress_bar:
            for x, y, labels in loader_train:
                model.train()  # put model to training mode
                x = x.to(device=device, dtype=dtype)  # move to device, e.g. GPU, shape: N, C, H, W
                y = y.to(device=device, dtype=dtype) # TODO: remember to change back, shape: N, 32, 2
                labels = labels.to(device=device, dtype=torch.long)
                if args.step == 'classification':
                    if args.ground_truth == 1:
                        scores = model(x, y)
                    else:
                        joints = model2(x)
                        N, D = joints.shape
                        joints = joints.reshape(N, D, 1)
                        flags = torch.zeros_like(joints).to(device)
                        joints = torch.cat((joints, flags), dim=1) # N, D, 2
                        scores = model(x, joints)
                else:
                    scores = model(x)
                loss = None
                accuracy = None
                if args.step == 'classification':
                    loss = class_loss(scores, labels) # TODO: Change this according to objective.
                    # loss = class_loss(F.log_softmax(scores, dim=1), F.log_softmax(F.one_hot(labels, 20).type(dtype), dim=1))
                    accuracy = (torch.argmax(scores, dim=-1)==labels).sum() / y.shape[0]
                else:
                    loss = util.loc_loss(scores, y)
                    # accuracy = torch.Tensor([0])
                    correct_body_count, body_count = util.calc_pckh5_acc(scores, y)
                    part_accuracy = correct_body_count / body_count
                    accuracy = part_accuracy[7]
                    # loss = F.mse_loss(scores, y[:, :, 0])
                    # print(loss)

                # Zero out all of the gradients for the variables which the optimizer
                # will update.
                optimizer.zero_grad()

                # This is the backwards pass: compute the gradient of the loss with
                # respect to each  parameter of the model.
                loss.backward()

                # Actually update the parameters of the model using the gradients
                # computed by the backwards pass.
                optimizer.step()

                progress_bar.update(args.batch_size)
                progress_bar.set_postfix(epoch=e,
                                        loss=loss.item(),
                                        accuracy=accuracy.item())

                if timestep%20 == 0:
                    writer.add_scalar("Train/Loss", loss.item(), timestep) #timestep*y.shape[0]
                    writer.add_scalar("Train/Accuracy", accuracy.item(), timestep)
                timestep += 1
                # if t % args.print_every == 0:
                #     print('Iteration %d, loss = %.4f' % (t, loss.item()))
                steps_to_eval -= batch_size
                total_steps += batch_size
                if steps_to_eval <= 0:
                    steps_to_eval = args.eval_steps + steps_to_eval
                    loss_val, acc_val = evaluate(model, model2, loader_val, args.step, args.batch_size, eval_timestep)
                    metric_val = acc_val
                    if args.crit == 'loss':
                        metric_val = loss_val
                    eval_timestep += 1
                    saver.save(total_steps, model, metric_val, device)
                    print()
    #    lr_scheduler.step()
    # if e == 10:
    #     lr_scheduler.step()
    writer.flush()
    writer.close()

def evaluate(model, model2, loader_val, step, batch_size, eval_timestep):
    total_loss = 0
    total_acc = 0.0
    count = 0.0

    part_acc_count = torch.zeros(8).to(device=device, dtype=dtype)
    part_count = torch.zeros(8).to(device=device, dtype=dtype)

    with torch.no_grad(), \
                tqdm(total=len(loader_val.dataset)) as progress_bar:
        for x, y, labels in loader_val:
                model.eval()  # put model to training mode
                x = x.to(device=device, dtype=dtype)  # move to device, e.g. GPU
                y = y.to(device=device, dtype=dtype)
                labels = labels.to(device=device, dtype=torch.long)
                if step == 'classification':
                    if args.ground_truth == 1:
                        scores = model(x, y)
                    else:
                        joints = model2(x)
                        N, D = joints.shape
                        joints = joints.reshape(N, D, 1)
                        flags = torch.zeros_like(joints).to(device)
                        joints = torch.cat((joints, flags), dim=1) # N, D, 2
                        scores = model(x, joints)
                    total_acc += (torch.argmax(scores, dim=-1)==labels).sum()
                else:
                    scores = model(x)
                    correct_body_count, body_count = util.calc_pckh5_acc(scores, y)
                    part_acc_count += correct_body_count
                    part_count += body_count

                loss = None
                if step == 'classification':
                    loss = F.cross_entropy(scores, labels) # TODO: Change this according to objective.
                    # loss = F.kl_div(F.log_softmax(scores, dim=1), F.log_softmax(F.one_hot(labels, 20).type(dtype), dim=1), 
                            # reduction='batchmean', log_target=True)
                else:
                    loss = util.loc_loss(scores, y)
                    # loss = F.mse_loss(scores, y[:, :, 0])
                progress_bar.update(batch_size)
                progress_bar.set_postfix(loss=loss.item())
                total_loss += loss.item()
                count += 1.0
    total_loss_mean =  total_loss / count
    if step == 'classification':
        total_acc /= len(loader_val.dataset)
    else:
        total_part_acc = part_acc_count / part_count
        total_acc = total_part_acc[7]
    print('total_loss_mean:', total_loss_mean)
    print('Val acc:', total_acc.item())
    writer.add_scalar("Eval/Loss", total_loss / count , eval_timestep)
    writer.add_scalar("Eval/Accuracy", total_acc.item(), eval_timestep)
    return total_loss_mean, total_acc


def test_part(model, loader_test, args):
    print(f'Loading checkpoint from {args.load_path}...')
    model = util.load_model(model, args.load_path)
    args.save_dir = util.get_save_dir(args.save_dir, args.name, training=False)

    model = model.to(device)
    results = []
    total_loss = []
    num_pics = 100

    # The following are specific to joint accuracy
    part_acc_count = torch.zeros(8).to(device=device, dtype=dtype)
    part_count = torch.zeros(8).to(device=device, dtype=dtype)
    pckh5_acc = None

    with torch.no_grad(), \
                tqdm(total=len(loader_test.dataset)) as progress_bar:
        for x, y, labels in loader_test:
                model.eval()  # put model to eval mode
                x = x.to(device=device, dtype=dtype)  # move to device, e.g. GPU
                y = y.to(device=device, dtype=dtype)

                scores = model(x)
                loss = None
                if args.step == 'classification': # TODO: results id to class
                    scores = model(x) # ?
                    loss = F.cross_entropy(scores, y) # TODO: Change this according to objective.
                else:
                    loss = util.loc_loss(scores, y)
                    joints_output = scores
                    # scores = scores.view(-1, 14, 2) # reshape to N * 16 * 2 (y, x)
                    if num_pics > 0: #np.random.uniform(0., 1.) < 0.1 and 
                        draw_keypoints(joints_output[0,  :], x[0, :, :, :], args, num_pics)
                        num_pics -= 1
                    correct_body_count, body_count = util.calc_pckh5_acc(scores, y)
                    part_acc_count += correct_body_count
                    part_count += body_count
    pckh5_acc = part_acc_count / part_count
    print(pckh5_acc)  
    #                 scores[:, :, [0,1]] = scores[:, :, [1,0]] # (x, y)
    #                 z1 = torch.where(scores > 0, 1., 0.) # N * 14 * 2 boolean flag
    #                 z2 = torch.where(scores < 96, 1., 0.)
    #                 z = z1 * z2
    #                 z = torch.min(z, 2, keepdim=True).values # N * 14 * 1 boolean flag
    #                 trans_ratio = torch.cat(trans_ratio_w, trans_ratio_h).view(-1, 1, 2) # N * 1 * 2
    #                 arr = torch.cat((scores / trans_ratio, z), 2).cpu().numpy() # change x, y to original size, 14 * 3 result
    #                 results.append(arr)
    #             progress_bar.update(args.batch_size)
    #             progress_bar.set_postfix(loss=loss.item())
    #             total_loss.append(loss.item())
    
    # if args.step == 'classification': # TODO: return for classification
    #     pass
    # else:
    #     # Save file for localization
    #     results_arr = results[0] # N * 14 * 3
    #     for i in range(1, len(results)):
    #         results_arr = np.stack((results_arr, results[i]), 0)
    #     print(results_arr.shape)
    #     results = np.transpose(results, (1, 2, 0)) # 14 * 3 * N
    #     sub_path = join(args.save_dir, args.split + '_' + args.sub_file)
    #     data = {'joints':results}
    #     print(f'Writing file to {sub_path}...')
    #     savemat(sub_path, data, appendmat=False)

    # print('Total loss:', total_loss/len(total_loss))

# testing
if __name__ == '__main__':
    a = np.random.normal(0, 0.1, (14, 3, 100))
    data = {'a': a}
    path = '../test.mat'
    savemat(path, data, appendmat=False)
