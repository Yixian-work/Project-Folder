from preprocess import *

data_dir = "..\data\lsp_dataset"
trainloader, valloader, testloader = load_data_and_preprocess_leeds(data_dir)

print(len(trainloader), len(valloader), len(testloader))
t, (x, y), trans = trainloader[0]
print(t)
print(x)
print(x.data)
print(y)
print(y.data)
print(trans)