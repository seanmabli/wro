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

self.Info[0, :, :]: Board Colors
self.Info[1, :, :]: Location Occupied By Object

Does not work if 
'''

RED, GREEN, BLUE = 0, 1, 2

class Location:
  def __init__(self):
    self.Info = np.zeros((4, 3, 4))
    self.Info[0, :, :] = np.array([[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]])

  def SetLocation(self, RunNum, InputColor):
    self.Order = np.zeros((3, 4))
    for k in range(3):
      for j in range(4):
        for i in range(2):
          if(self.Order[i + RunNum, j] == 0):
            self.Order[i + RunNum, j] += np.where(self.Info[0, i + RunNum, j] == InputColor[k], 1, 0)
            
            if(self.Info[0, i + RunNum, j] == InputColor[k]):
              break
        else:
          continue
        break

  def SetLocationAsOccupied(Coordinates):
    self.Info[1, Coordinates[0], Coordinates[1]] = 1

  def SetObjectAsDelivered(self, Coordinates):
    self.Order -= 1
    self.Order = np.clip(self.Order, a_min=0)
    SetLoactionAsOccupied(Coordinates)

Loc = Location()

Loc.SetLocation(1, np.array([RED, RED, BLUE]))

print(Loc.GetOrder())