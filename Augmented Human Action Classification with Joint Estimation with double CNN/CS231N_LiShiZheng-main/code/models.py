import torch
import torch.nn as nn
from util import *
import torchvision
import torch.nn.functional as F
from torchvision.models import resnet18
# from torchvision.models.feature_extraction import create_feature_extractor,get_graph_node_names

# Simple model
class Simple(nn.Module):
    def __init__(self, drop_prob=0.1):
        super(Simple, self).__init__()
        self.my_layer = nn.Sequential(
                        nn.Conv2d(3, 64, 5, padding=2),
                        nn.Dropout2d(drop_prob),
                        nn.ReLU(),
                        nn.Conv2d(64, 128, 3, padding=1),
                        nn.ReLU(),
                        Flatten(),
                        nn.Linear(128 * 32 * 32, 10),
                    )
    
    def forward(self, data):
        out = self.my_layer(data)
        return out

class CRPD(nn.Module): # Conv-ReLU-Pool-Dropout
    def __init__(self, drop_prob=0.1):
        super(CRPD, self).__init__()
        self.layer = nn.Sequential(
                    nn.Conv2d(3, 32, 3, padding=1),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.MaxPool2d(2), # 32 * 112 * 112
                    nn.Conv2d(32, 64, 3, padding=1),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.MaxPool2d(2), # 64 * 56 * 56
                    nn.Conv2d(64, 128, 3, padding=1),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.MaxPool2d(2), # 128 * 28 * 28
                    nn.Conv2d(128, 128, 3, padding=1),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.MaxPool2d(2), # 128 * 14 * 14
                    Flatten(), 
                    nn.BatchNorm1d(128 * 14 * 14),
                    nn.Linear(128 * 14 * 14, 4086),
                    nn.ReLU(),
                    nn.Dropout(drop_prob),
                    nn.BatchNorm1d(4086),
                    nn.Linear(4086, 1024),
                    nn.ReLU(),
                    nn.Dropout(drop_prob),
                    nn.BatchNorm1d(1024),
                    nn.Linear(1024, 1024),
                    nn.ReLU(),
                    nn.Dropout(drop_prob),
                    nn.BatchNorm1d(1024),
                    nn.Linear(1024, 32),
                    )
    
    def forward(self, data):
        out = self.layer(data)
        return out

class DeepPose(nn.Module):
	"""docstring for DeepPose"""
	def __init__(self, nJoints, modelName='resnet34'):
		super(DeepPose, self).__init__()
		self.nJoints = nJoints
		self.block = 'BottleNeck' if (int(modelName[6:]) > 34) else 'BasicBlock'
		self.resnet = getattr(torchvision.models, modelName)(pretrained=True)
		self.resnet.fc = nn.Linear(512 * (4 if self.block == 'BottleNeck' else 1), self.nJoints * 2)
	def forward(self, x):
		return self.resnet(x)

class nn_Classifier(nn.Module):
    def __init__(self, drop_prob=0.2):
        super(nn_Classifier, self).__init__()
        self.feature_extractor = shallow_cnn_Feature_extractor() #(N,113)
        # self.joints_feature_extactor = joints_feature_extactor() #(N,15)
        self.joints_feature_extactor = conv_joints_fe()
        self.classifer = nn.Sequential(
                        nn.Linear(256,256),
                        nn.ReLU(),
                        # nn.BatchNorm1d(64),
                        nn.Linear(256, 256),
                        nn.Dropout2d(drop_prob),
                        nn.ReLU(),
                        # nn.BatchNorm1d(64),
                        nn.Linear(256,15),
                        # nn.Softmax() #could be deleted
                    )
    
    def forward(self, data, joints):
        
        features = self.feature_extractor(data)

        joints_processed = torch.mul(joints[:,:,0], joints[:,:,1]).view(-1,32) #N*32

        joints_features = self.joints_feature_extactor(joints_processed) #N* 256
        # print(features.shape, joints_features.shape)

        features = torch.cat((features, joints_features), dim=-1) #N * 512
        # print(features.shape)
        
        out = self.classifer(features)
        return out


vision_extractor = resnet18(pretrained=True)
class Resnet_Feature_extractor(nn.Module):
    def __init__(self):
        super().__init__()
        self.feature_extractor = nn.Sequential(
                    # stop at conv4
                    *list(vision_extractor.children())[:-3],
                    Flatten(),
                    nn.Linear(50176, 512)
                )

    def forward(self, x):
        feats = self.feature_extractor(x)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return feats

class joints_feature_extactor(nn.Module):
    def __init__(self):
        super().__init__()
        self.model = nn.Sequential(
            nn.Linear(32, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, 128),
        )

    def forward(self, x):
        return self.model(x)

class conv_joints_fe(nn.Module):
    def __init__(self):
        super(conv_joints_fe, self).__init__()
        self.model = nn.Sequential(
            nn.Conv1d(2, 64, 3),
            nn.ReLU(),
            nn.Conv1d(64, 64, 3),
            nn.ReLU(),
            nn.MaxPool1d(2),
            Flatten(),
            nn.Linear(384, 128),
            # nn.ReLU(),
            # nn.Linear(128, 128)
        )
    def forward(self, x):
        N, D = x.shape
        x_x = torch.zeros(N, D//2)
        x_y = torch.zeros(N, D//2)
        idx = 0
        for i in range(D):
            if i % 2 == 0:
                x_x[:, idx] = x[:, i]
            else:
                x_y[:, idx] = x[:, i]
                idx += 1
        x_x.unsqueeze_(1)
        x_y.unsqueeze_(1)
        x_r = torch.cat((x_y, x_x), dim=1).to(torch.device('cuda'))
        # print(x_r.shape)
        # print(x_r)
        return self.model(x_r)

class cnn_Feature_extractor(nn.Module):
    def __init__(self, drop_prob=0.2):
        super().__init__()
        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, 32, 4),
            nn.ReLU(),
            nn.MaxPool2d(2),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(32, 64, 4),
            nn.ReLU(),
            nn.MaxPool2d(2),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(64, 64, 4),
            nn.ReLU(),
            nn.MaxPool2d(2),  
            nn.Dropout2d(drop_prob),
            Flatten(),
            nn.Linear(40000, 256)
            )

    def forward(self, x):
        feats = self.feature_extractor(x)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return feats

class cnn_classifier(nn.Module):
    def __init__(self, drop_prob=0.2):
        super().__init__()
        self.classifer = nn.Sequential(
            nn.Conv2d(3, 32, 5),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(4),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(32, 64, 5),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(3),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(64, 64, 3, padding=1),
            nn.ReLU(), 
            nn.Dropout2d(drop_prob),
            # nn.Conv2d(64, 64, 5),
            # nn.ReLU(),
            # nn.MaxPool2d(2),  
            # nn.Dropout2d(drop_prob),
            Flatten(),
            nn.Linear(18496, 4096),
            nn.ReLU(),
            nn.BatchNorm1d(4096),
            # nn.Linear(256,256),
            # nn.Dropout2d(drop_prob),
            # nn.ReLU(),
            # nn.BatchNorm1d(256), 
            nn.Linear(4096, 1024),
            nn.Dropout2d(drop_prob),
            nn.ReLU(),
            nn.BatchNorm1d(1024),
            nn.Linear(1024,20)
            )

    def forward(self, x, joints):
        preds = self.classifer(x)
        # print(preds.shape)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return preds

# vision_extractor = resnet18(pretrained=True)
class Resnet_Classifier(nn.Module):
    def __init__(self):
        super().__init__(drop_prob=0.2)
        self.classifer = nn.Sequential(
                    # stop at conv4
                    *list(vision_extractor.children())[:-3],
                    Flatten(),
                    nn.Linear(50176, 256),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.BatchNorm1d(256),
                    nn.Linear(256,128),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.BatchNorm1d(128), 
                    nn.Linear(128, 64),
                    nn.Dropout2d(drop_prob),
                    nn.ReLU(),
                    nn.BatchNorm1d(64),
                    nn.Linear(64,20),
                )

    def forward(self, x):
        preds = self.classifer(x)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return preds

class shallow_cnn_classifier(nn.Module):
    def __init__(self, drop_prob=0.2):
        super().__init__()
        self.classifer = nn.Sequential(
            nn.BatchNorm2d(3),
            nn.Conv2d(3, 32, 4),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(4),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(32, 16, 4),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.MaxPool2d(3),  
            nn.Dropout2d(drop_prob),
            Flatten(),
            nn.Linear(4624, 128),
            nn.ReLU(),
            nn.BatchNorm1d(128),
            nn.Linear(128,15)
            )

    def forward(self, x, joints):
        preds = self.classifer(x)
        print(preds.shape)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return preds

class shallow_cnn_Feature_extractor(nn.Module):
    def __init__(self, drop_prob=0.2):
        super().__init__()
        self.feature_extractor = nn.Sequential(
            nn.BatchNorm2d(3),
            nn.Conv2d(3, 32, 4),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(4),  
            nn.Dropout2d(drop_prob),
            nn.Conv2d(32, 16, 4),
            nn.ReLU(),
            nn.BatchNorm2d(16),
            nn.MaxPool2d(3),  
            nn.Dropout2d(drop_prob),
            Flatten(),
            nn.Linear(4624, 128),
            # nn.ReLU(),
            # nn.BatchNorm1d(128),
            )

    def forward(self, x):
        feats = self.feature_extractor(x)
        # here is just adjust the shape of features to be (..., 512), delete redundant dim
        # feats = feats.squeeze(3).squeeze(2)
        
        # final dim is (..., 512), may add another fc layer to change the output dim
        return feats
