import numpy as np
'''
Input Color:
Red = 0, Green = 1, Blue = 2

Board Layout:
R | R | G | B
G | R | G | B
B | B | R | G

Run Number:
0 = 
R | R | G | B
G | R | G | B

1 = 
G | R | G | B
B | B | R | G

Board Colors: Info[0, :, :]
'''
Info = np.zeros((4, 3, 4))
Info[0, :, :] = np.array([[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]])

def findLocation(RunNum, InputColor):
  Order = np.zeros((3, 4))
  for i in range(3):
    for j in range(4):
      if(str(np.where(Info[0, RunNum:RunNum + 2, j] == InputColor[i])) != '(array([], dtype=int64),)'):
        Order[int(str(np.where(Info[0, RunNum:RunNum + 2, j] == InputColor[i]))[8:9]) + RunNum , j] = i + 1
        break
  return Order

print(findLocation(1, np.array([0, 1, 2])))