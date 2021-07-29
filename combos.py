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
'''

RED, GREEN, BLUE = 0, 1, 2

class Location:
  def __init__(self):
    self.BoardColor = [[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]]
    self.LocOccupied = [[0]*4 for _ in range(3)]
    self.Order = [[0]*4 for _ in range(3)]

  def SetLocation(self, RunNum, InputColor):
    for k in range(3):
      for j in range(4):
        for i in range(2):
          if(self.Order[i + RunNum][j] == 0 and self.BoardColor[i + RunNum][j] == InputColor[k]):
            self.Order[i + RunNum][j] = 1
            break
        else:
          continue
        break

  def SetLocationAsOccupied(self, Coordinates):
    self.LocOccupied[Coordinates[0]][Coordinates[1]] = 1

  def SetLocationAsDelivered(self, Coordinates):
    self.LocOccupied[Coordinates[0]][Coordinates[1]] = 1
    self.Order[Coordinates[0]][Coordinates[1]] = 0

Loc = Location()
Loc.SetLocation(1, [BLUE, BLUE, BLUE])

print(Loc.Order)
Loc.SetLocationAsDelivered([2, 0])
print(Loc.Order)