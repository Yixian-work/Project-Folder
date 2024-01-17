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

def getTrainArgs():

    parser = argparse.ArgumentParser('Train a model on joint localization/classification')

    parser.add_argument('--step',
                        type=str,
                        default='localization',
                        choices=('localization', 'classification'),
                        help='Determine which step to train')
    parser.add_argument('--eval_steps',
                        type=int,
                        default=50000,
                        help='Number of steps between successive evaluations.')
    parser.add_argument('--lr',
                        type=float,
                        default=0.0001,
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
                        default=128,
                        help='Batch size.')
    parser.add_argument('--seed',
                        type=int,
                        default=2,
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
    parser.add_argument('--max_checkpoints',
                        type=int,
                        default='10',
                        help='Max number of checkpoints to save.')
    parser.add_argument('--ground_truth',
                        type=int,
                        default=1,
                        help='Whether to use gound truth joints')
    parser.add_argument('--crit',
                        type=str,
                        default='loss',
                        choices=('loss','acc'),
                        help='criteria for saving model')
    parser.add_argument('--load_path',
                        type=str,
                        default=None,
                        help='Path to load as a model checkpoint. Used for classification when ground_truth=0.')
    
    args = parser.parse_args()
    return args

def main(args):
    dtype = torch.float32 # We will be using float throughout this tutorial.
    if args.ground_truth == 0 and args.load_path == None:
        raise argparse.ArgumentError('Missing required argument --load_path')

    data_dir, loader_train, loader_val, loader_test = None, None, None, None # TODO: add img directory
    
    print('Preprocessing data...')
    if args.data_dir == 'lsp':
        data_dir = '../data/lspet_dataset'
        loader_train, loader_val, loader_test = load_data_and_preprocess_leeds(data_dir, args.batch_size)
    else:
        data_dir = '../data/data-mpii' # TODO: Download and check
        labels_check = np.zeros(20)
        loader_train, loader_val= load_data_and_preprocess_MPII(labels_check, data_dir, args.batch_size)
        print("counts for labels")
        print(labels_check)
    # Set random seed
    print(f'Using random seed {args.seed}...')
    # np.random.seed(args.seed)
    # torch.manual_seed(args.seed)
    # torch.cuda.manual_seed_all(args.seed)

    # TODO: Define our sequential model here. Below is example
    model_loc = CRPD()
    model_class = nn_Classifier(drop_prob=args.drop_prob)

    # TODO: Training our network
    step = args.step
    if step == 'localization':
        # TODO: Define our optimizer to train
        lr_scheduler = None
        optimizer = optim.SGD(model_loc.parameters(), lr=args.lr,
                        momentum=0.9, nesterov=True, weight_decay=args.l2_wd)
        train_part(model_loc, optimizer, loader_train, loader_val, args, lr_scheduler)
    elif step == 'classification':
        # TODO: Define our optimizer to train
        # optimizer = optim.SGD(model_class.parameters(), lr=args.lr,
                        # momentum=0.9, nesterov=True, weight_decay=args.l2_wd)
        optimizer = optim.Adam(model_class.parameters(), lr=args.lr,
                        weight_decay=args.l2_wd)
        decayRate = 0.1
        lr_scheduler = torch.optim.lr_scheduler.MultiStepLR(optimizer=optimizer, milestones=[10], gamma=decayRate)
        train_part(model_class, optimizer, loader_train, loader_val, args, lr_scheduler)
    


if __name__ == '__main__':
    main(getTrainArgs())
