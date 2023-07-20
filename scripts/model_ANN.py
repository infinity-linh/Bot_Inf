import torch.nn as nn
import tqdm
import torch
class ANN(nn.Module):
    def __init__(self, input=4):
        super().__init__()

        # self.relu1 = nn.ReLU(inplace=True)
        self.liner1 = nn.Linear(input,128)
        self.relu = nn.ReLU()
        self.liner2 = nn.Linear(128,8)
        self.liner3 = nn.Linear(8,4)

        # self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        out = self.relu(self.liner1(x))
        out = self.relu(self.liner2(out))
        out = self.relu(self.liner3(out))

        return out