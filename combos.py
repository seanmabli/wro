import numpy as np

Info = np.zeros((4, 3, 4))
Info[0, :, :] = np.array([[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]])

def FindLocation(Input):
  Order = np.zeros((3, 4))
  while (np.sum(Order) != 6):
    Info[0, 1:, :]