import torch
import torchvision
x = torch.rand(5, 3)
print(x)

# print(torch.zeros(1).cuda())

while True:
    print(torch.cuda.is_available())