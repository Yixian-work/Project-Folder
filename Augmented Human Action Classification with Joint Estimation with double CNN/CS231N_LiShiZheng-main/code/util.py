import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import os
import queue
import shutil

dtype = torch.float32 # We will be using float throughout this tutorial.

device = torch.device('cuda')

def flatten(x):
    N = x.shape[0] # read in N, C, H, W
    return x.view(N, -1)  # "flatten" the C * H * W values into a single vector per image

def random_weight(shape):
    if len(shape) == 2:  # FC weight
        fan_in = shape[0]
    else:
        fan_in = np.prod(shape[1:]) # conv weight [out_channel, in_channel, kH, kW]
    # randn is standard normal distribution generator. 
    w = torch.randn(shape, device=device, dtype=dtype) * np.sqrt(2. / fan_in)
    w.requires_grad = True
    return w

def zero_weight(shape):
    return torch.zeros(shape, device=device, dtype=dtype, requires_grad=True)

def get_save_dir(base_dir, name, training, id_max=100):
    """Get a unique save directory by appending the smallest positive integer
    `id < id_max` that is not already taken (i.e., no dir exists with that id).

    Args:
        base_dir (str): Base directory in which to make save directories.
        name (str): Name to identify this training run. Need not be unique.
        training (bool): Save dir. is for training (determines subdirectory).
        id_max (int): Maximum ID number before raising an exception.

    Returns:
        save_dir (str): Path to a new directory with a unique name.
    """
    for uid in range(1, id_max):
        subdir = 'train' if training else 'test'
        save_dir = os.path.join(base_dir, subdir, f'{name}-{uid:02d}')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            return save_dir

    raise RuntimeError('Too many save directories created with the same name. \
                       Delete old save directories or use another name.')

def loc_loss(scores, y):
    N, _, _ = y.shape
    y_locations = y[:, :, 0]
    y_flags = y[:, :, 1]
    # print(y_locations)
    # print(scores)
    p = torch.sum(y_flags) / N
    loss = torch.mean(((scores - y_locations) ** 2) * y_flags) / p
    # print(loss)
    return loss

# pck5 values for two parts (left and right), each part is a vector of 2 elements: x and y
def pck5_two_parts(part1, part2, ground1, ground2, flag1, flag2, threshold):
    if flag1 != 0 and flag2 != 0:
        acc = 0.5 * (torch.linalg.norm(part1 - ground1) <= threshold) + 0.5 * (torch.linalg.norm(part2 - ground2) <= threshold)
    elif flag1 == 0 and flag2 != 0:
        acc = (torch.linalg.norm(part2 - ground2) <= threshold)
    elif flag1 != 0 and flag2 == 0:
        acc = (torch.linalg.norm(part1 - ground1) <= threshold)
    else:
        acc = None
    return acc

def calc_pckh5_acc(scores, y):
    N, _, _ = y.shape
    y_locations = y[:, :, 0]
    y_flags = y[:, :, 1]
    # body count and correct body count
    body_count = torch.zeros(8).to(device=device, dtype=dtype)
    correct_body_count = torch.zeros(8).to(device=device, dtype=dtype)
    # Loop through exmaple (naive!)
    for i in range(N):
        score = scores[i,:]
        y_location = y_locations[i,:]
        y_flag = y_flags[i,:]
        # Calculate the headsize and threshold (utilize head: id:9 and upper neck: id:8), skip if invisible.
        threshold = None
        if y_flag[2*9] == 0 or y_flag[2*8] == 0:
            continue
        else:
            headsize = torch.linalg.norm(y_location[9*2:9*2+1] - y_location[8*2:8*2+1])
            # Calculate the threshold based on the headsize
            threshold = headsize * 0.5 # According to the pckh@0.5 standard
        # Head (id:9, there exists a valid one)
        if torch.linalg.norm(y_location[2*9:2*9+1] - score[2*9:2*9+1]) <= threshold:
            correct_body_count[0] += 1
            correct_body_count[7] += 1
        body_count[0] += 1
        body_count[7] += 1
        # Shoulder (rid:12,lid:13)
        acc_sh = pck5_two_parts(score[2*12:2*12+1], score[2*13:2*13+1], y_location[2*12:2*12+1], y_location[2*13:2*13+1], y_flag[2*12], y_flag[2*13], threshold)
        if acc_sh is not None:
            correct_body_count[1] += acc_sh # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_sh
            body_count[1] += 1
            body_count[7] += 1
        # Elbow (rid:11,lid:14)
        acc_el = pck5_two_parts(score[2*11:2*11+1], score[2*14:2*14+1], y_location[2*11:2*11+1], y_location[2*14:2*14+1], y_flag[2*11], y_flag[2*14], threshold)
        if acc_el is not None:
            correct_body_count[2] += acc_el # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_el
            body_count[2] += 1
            body_count[7] += 1
        # Wrist (rid:10,lid:15)
        acc_wr = pck5_two_parts(score[2*10:2*10+1], score[2*15:2*15+1], y_location[2*10:2*10+1], y_location[2*15:2*15+1], y_flag[2*10], y_flag[2*15], threshold)
        if acc_wr is not None:
            correct_body_count[3] += acc_wr # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_wr
            body_count[3] += 1
            body_count[7] += 1
        # Hip (rid:2,lid:3)
        acc_hi = pck5_two_parts(score[2*2:2*2+1], score[2*3:2*3+1], y_location[2*2:2*2+1], y_location[2*3:2*3+1], y_flag[2*2], y_flag[2*3], threshold)
        if acc_hi is not None:
            correct_body_count[4] += acc_hi # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_hi
            body_count[4] += 1
            body_count[7] += 1
        # Knee (rid:1,lid:4)
        acc_kn = pck5_two_parts(score[2*1:2*1+1], score[2*4:2*4+1], y_location[2*1:2*1+1], y_location[2*4:2*4+1], y_flag[2*1], y_flag[2*4], threshold)
        if acc_kn is not None:
            correct_body_count[5] += acc_kn # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_kn
            body_count[5] += 1
            body_count[7] += 1
        # Ankle (rid:0,lid:5)
        acc_an = pck5_two_parts(score[2*0:2*0+1], score[2*5:2*5+1], y_location[2*0:2*0+1], y_location[2*5:2*5+1], y_flag[2*0], y_flag[2*5], threshold)
        if acc_an is not None:
            correct_body_count[6] += acc_an # Notice this value could either be 0, 0.5, or 1
            correct_body_count[7] += acc_an
            body_count[6] += 1
            body_count[7] += 1

    return correct_body_count, body_count



class CheckpointSaver:
    """Class to save and load model checkpoints.

    Save the best checkpoints as measured by a metric value passed into the
    `save` method. Overwrite checkpoints with better checkpoints once
    `max_checkpoints` have been saved.

    Args:
        save_dir (str): Directory to save checkpoints.
        max_checkpoints (int): Maximum number of checkpoints to keep before
            overwriting old ones.
    """
    def __init__(self, save_dir, max_checkpoints, criteria):
        super(CheckpointSaver, self).__init__()

        self.save_dir = save_dir
        self.max_checkpoints = max_checkpoints
        self.best_val = None
        self.ckpt_paths = queue.PriorityQueue()
        self.criteria = criteria

    def is_best(self, metric_val):
        """Check whether `metric_val` is the best seen so far.

        Args:
            metric_val (float): Metric value to compare to prior checkpoints.
        """
        if metric_val is None:
            # No metric reported
            return False

        if self.best_val is None:
            # No checkpoint saved yet
            return True
        if self.criteria=='loss':
            return self.best_val > metric_val
        else:
            return metric_val > self.best_val

    def save(self, step, model, metric_val, device):
        """Save model parameters to disk.

        Args:
            step (int): Total number of examples seen during training so far.
            model (torch.nn.Module): Model to save.
            metric_val (float): Determines whether checkpoint is best so far.
            device (torch.device): Device where model resides.
        """
        ckpt_dict = {
            'model_name': model.__class__.__name__,
            'model_state': model.cpu().state_dict(),
            'step': step
        }
        model.to(device)

        checkpoint_path = os.path.join(self.save_dir,
                                       f'step_{step}.pth.tar')
        torch.save(ckpt_dict, checkpoint_path)
        print(f'Saved checkpoint: {checkpoint_path}')

        if self.is_best(metric_val):
            # Save the best model
            self.best_val = metric_val
            best_path = os.path.join(self.save_dir, 'best.pth.tar')
            shutil.copy(checkpoint_path, best_path)
            print(f'New best checkpoint at step {step}...')

        # Add checkpoint path to priority queue (lowest priority removed first)
        priority_order = -metric_val

        self.ckpt_paths.put((priority_order, checkpoint_path))

        # Remove a checkpoint if more than max_checkpoints have been saved
        if self.ckpt_paths.qsize() > self.max_checkpoints:
            _, worst_ckpt = self.ckpt_paths.get()
            try:
                os.remove(worst_ckpt)
                print(f'Removed checkpoint: {worst_ckpt}')
            except OSError:
                # Avoid crashing if checkpoint has been removed or protected
                pass

class Flatten(nn.Module):
    def forward(self, x):
        return flatten(x)

def load_model(model, checkpoint_path):
    """Load model parameters from disk.

    Args:
        model (torch.nn.Module): Load parameters into this model.
        checkpoint_path (str): Path to checkpoint to load.

    Returns:
        model (torch.nn.Module): Model loaded from checkpoint.
    """
    ckpt_dict = torch.load(checkpoint_path)

    # Build model, load parameters
    model.load_state_dict(ckpt_dict['model_state'])

    return model

# Dummy test for loc_loss
if __name__ == '__main__':
    scores = torch.normal(0, 1, size=(50, 14))
    y = torch.normal(5, 1, size=(50, 14, 2))
    loss = loc_loss(scores, y)
    print(loss)