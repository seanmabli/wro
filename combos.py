import numpy as np

Info = np.zeros((4, 3, 4))
Info[0, :, :] = np.array([[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]])

def findLocation(Input):
  Order = np.zeros((3, 4))
  if(np.where(Info[0, 1:, 0] == Input) != []):
    Order[int(str(np.where(Info[0, 1:, 1] == Input))[8:9]) + 1, 1] = 1
  return Order

print(findLocation(0))