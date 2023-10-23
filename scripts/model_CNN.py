import torch.nn as nn
import cv2
import numpy as np
import torch

class Net(nn.Module):
    
    def __init__(self, im_input=256):
        super(Net, self).__init__()
        
        self.layer1 = nn.Sequential(
            nn.Conv2d(in_channels=3, out_channels=32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)
        )
        
        self.layer2 = nn.Sequential(
            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2)
        )
        
        self.fc1 = nn.Linear(in_features=64*(im_input//4-1)**2, out_features=512)
        self.drop = nn.Dropout2d(0.25)
        self.fc2 = nn.Linear(in_features=512, out_features=128)
        self.fc3 = nn.Linear(in_features=128, out_features=4)
        self.out = nn.Sigmoid()
        
    def forward(self, x, y):
        out1 = self.layer1(x)
        out2 = self.layer1(y)

        out1 = self.layer2(out1)
        out2 = self.layer2(out2)

        out1 = out1.view(out1.size(0), -1)
        out2 = out2.view(out2.size(0), -1)

        out1 = self.fc1(out1)
        out2 = self.fc1(out2)

        out1 = self.drop(out1)
        out2 = self.drop(out2)
        
        out1 = self.fc2(out1)
        out2 = self.fc2(out2)
        out = out1 + out2
        out = self.fc3(out)
        # out2 = self.fc3(out2)
        
        return out
    
# image1 = cv2.imread("D:/User/data_map/right/imager_0.png")
# image2 = cv2.imread("D:/User/data_map/left/imagel_0.png")

# image1 = cv2.resize(image1, [256,256]).transpose([2,0,1])/256
# image2 = cv2.resize(image2, [256,256]).transpose([2,0,1])/256

# net = Net(im_input=256)
# # print(image.shape)
# im1 = torch.tensor(np.array([image1]),dtype=torch.float32)
# im2 = torch.tensor(np.array([image2]),dtype=torch.float32)

# print(net(im1, im2))