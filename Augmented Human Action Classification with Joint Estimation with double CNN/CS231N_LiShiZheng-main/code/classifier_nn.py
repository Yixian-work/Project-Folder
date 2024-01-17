import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import torch.nn as nn
# from util import *
from torchvision.models import resnet18
# from torchvision.models.feature_extraction import create_feature_extractor,get_graph_node_names
import torch
from torch import nn

class nn_Classifier(nn.Module):
    def __init__(self, feature_extractor, joints_feature_extactor):
        super(nn_Classifier, self).__init__()
        self.feature_extractor = feature_extractor
        self.joints_feature_extactor = joints_feature_extactor
        self.classifer = nn.Sequential(
                        nn.Linear(512+512,64),
                        nn.ReLU(),
                        nn.BatchNorm1d(64),
                        
                        nn.Linear(64, 64),
                        nn.ReLU(),
                        nn.BatchNorm1d(64),
                        
                        nn.Linear(64,20),
                        nn.Softmax() #could be deleted
                    )
    
    def forward(self, data, joints):
        
        features = self.feature_extractor(data)

        joints_processed = torch.mul(joints[:,:,0], joints[:,:,1]).view(-1,32) #N*32

        joints_features = self.joints_feature_extactor(joints_processed) #N* 512

        features = torch.cat((features, joints), -1) #N * 1024
        
        out = self.classifer(data)
        return out

vision_extractor = resnet18(pretrained=True)
class Resnet_Feature_extractor(nn.Module):
    def __init__(self, feature_extractor):
        super().__init__()
        self.feature_extractor = nn.Sequential(
                    # stop at conv4
                    *list(vision_extractor.features.children())[:-3]
                )

    def forward(self, x):
        feats = self.feature_extractor(x)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return feats

def feature_extractor():
    
    return Resnet_Feature_extractor()


class joints_feature_extactor(nn.Module):
    def __init__(self, feature_extractor):
        super().__init__()
        self.model = nn.Sequential(
            nn.Linear(32, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, 512),
        )

    def forward(self, x):
        return self.forward(x)