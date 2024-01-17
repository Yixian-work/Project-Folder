import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F  # useful stateless functions
import numpy as np
from models import *
from preprocess import *
from train import *
from util import *
import argparse

def getTestArgs():

    parser = argparse.ArgumentParser('Train a model on joint localization/classification')

    parser.add_argument('--step',
                        type=str,
                        default='localization',
                        choices=('localization', 'classification'), #TODO: maybe add 'both' when mpii
                        help='Determine which step to train')
    parser.add_argument('--eval_steps',
                        type=int,
                        default=50000,
                        help='Number of steps between successive evaluations.')
    parser.add_argument('--lr',
                        type=float,
                        default=0.5,
                        help='Learning rate.')
    parser.add_argument('--l2_wd',
                        type=float,
                        default=0,
                        help='L2 weight decay.')
    parser.add_argument('--num_epochs',
                        type=int,
                        default=30,
                        help='Number of epochs for which to train.')
    parser.add_argument('--drop_prob',
                        type=float,
                        default=0.2,
                        help='Probability of zeroing an activation in dropout layers.')
    parser.add_argument('--name',
                        '-n',
                        type=str,
                        required=True,
                        help='Name to identify training or test run.')
    parser.add_argument('--save_dir',
                        type=str,
                        default='../save/',
                        help='Base directory for saving information.')
    parser.add_argument('--batch_size',
                        type=int,
                        default=32,
                        help='Batch size.')
    parser.add_argument('--seed',
                        type=int,
                        default=231,
                        help='Random seed for reproducibility.')
    parser.add_argument('--print_every',
                        type=int,
                        default=1000,
                        help='Iterations between status print.')
    parser.add_argument('--data_dir',
                        type=str,
                        default='mpii',
                        choices=('lsp','mpii'),
                        help='Input data location')
    parser.add_argument('--split',
                        type=str,
                        default='val',
                        choices=('train', 'val'),
                        help='Split to use for testing.')
    parser.add_argument('--sub_file',
                        type=str,
                        default='joints.mat',
                        help='Name for submission file.')
    parser.add_argument('--load_path',
                        type=str,
                        default=None,
                        help='Path to load as a model checkpoint.')

    # Require load_path for test.py
    args = parser.parse_args()
    if not args.load_path:
        raise argparse.ArgumentError('Missing required argument --load_path')
    

    return args

def main(args):
    dtype = torch.float32 # We will be using float throughout this tutorial.

    data_dir, loader_train, loader_val, loader_test = None, None, None, None # TODO: add img directory
    labels_check = np.zeros(20)
    print('Preprocessing data...')
    if args.data_dir == 'lsp':
        data_dir = '../data/lspet_dataset'
        loader_train, loader_val, loader_test = load_data_and_preprocess_leeds(data_dir, args.batch_size)
    else:
        data_dir = '../data/data-mpii' # TODO: Download and check
        loader_train, loader_val = load_data_and_preprocess_MPII(labels_check, data_dir, args.batch_size)

    # Set random seed
    print(f'Using random seed {args.seed}...')
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)

    # TODO: Define our sequential model here. Below is example
    model_loc = CRPD()
    model_class = Simple(args.drop_prob)

    # TODO: Training our network
    step = args.step
    split = args.split
    data_loader = None
    if split == 'train':
        data_loader = loader_train
    else:
        data_loader = loader_val
    if step == 'localization':
        test_part(model_loc, data_loader, args)
    elif step == 'classification':
        train_part(model_class, data_loader, args)
    


if __name__ == '__main__':
    main(getTestArgs())