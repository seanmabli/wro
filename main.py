#!/usr/bin/env pybricks-micropython

# Color Scanning
'''
Color.RED = 0
Color.GREEN = 1
Color.BLUE = 2

Color.BLACK = 3
Color.YELLOW = 4
Color.WHITE = 5
Color.BROWN = 6
None = 7
'''

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time

StartTime = time.time()

def LastCar(CarColor):
  return 6 - sum(CarColor) # 1 + 1 + 2 + 2 = 6

def MotorHold():
  robot.stop()
  LeftMotor.hold()
  RightMotor.hold()

def LineSquaring(Num):
  THRESHOLD = (9 + 70) / 2
  # Move Forward To White
  while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
    robot.drive(-50 * Num, 0)

  MotorHold()

  # If Left Color Detected The White
  if(LeftColor.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while LeftColor.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    MotorHold()
      
  # If Right Color Detected The White
  if(RightColor.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while RightColor.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    MotorHold()

  # Move Left Or Right To Get Color Reflection Equal
  while RightColor.reflection() > LeftColor.reflection():
    RightMotor.run(-10 * Num)
    LeftMotor.run(10 * Num)
  MotorHold()
  while RightColor.reflection() < LeftColor.reflection():
    RightMotor.run(10 * Num)
    LeftMotor.run(-10 * Num)
  MotorHold()

def LineFollowingDistance(Distance, Sensor, ProportionalGain):
  # Circumfrence = 174.36 mm
  # Distance in cm x 10 -> mm
  Distance = Distance * 10

  Threshold = (9 + 70) / 2 # Black = 9, White = 70
  LeftMotor.reset_angle(0)
  RightMotor.reset_angle(0)
  while abs((LeftMotor.angle() + RightMotor.angle()) / 2) < (Distance / 174.36) * 360:
    if(Sensor == 'Left'):
      Deviation = (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(-100, turn_rate)

  MotorHold()

def LineFollowingToBlack(Sensor, ProportionalGain):
  Threshold = (9 + 70) / 2 # Black = 9, White = 70
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    if(Sensor == 'Left'):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(-100, turn_rate)

  MotorHold()

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

'''
def BayOperation(Bay, Operation): # Close if Operation = -1 and Open if Operation = 1
  if Bay == 0: # All

  elif Bay == 1: # Bay 1
  elif Bay == 2: # Bay 2
  elif Bay == 3: # Bay 3
    
  elif Bay == 4: # Left Arm (Covers 1.5 Bays)
  elif Bay == 3: # Right Arm (Covers 1.5 Bays)
'''

LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)
LeftArm = Motor(Port.D)
RightArm = Motor(Port.A)

LeftColor = ColorSensor(Port.S3)
RightColor = ColorSensor(Port.S2)
SideColor = ColorSensor(Port.S1)
UltraSonic = UltrasonicSensor(Port.S4)
robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=55.5, axle_track=139)
robot.settings(straight_speed=200, turn_rate=65)

FirstColorScan = [0] * 6
SecColorScan = [0] * 6
ColorScan = [0] * 6

RightMotor.run_target(400, -300)
LeftMotor.run_target(400, -300)
while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
  robot.drive(-200, 0)
robot.straight(-255)
robot.turn(-80)
robot.straight(150)
LineSquaring(-1)
robot.turn(3) # Squaring always is angleded to the left so this should counter that
robot.straight(35)

for i in range(5):
  robot.straight(115)
  FirstColorScan[i] = SideColor.color()

for i in reversed(range(5)):
  SecColorScan[i] = SideColor.color()
  robot.straight(-115)

for i in range(5):
  if (FirstColorScan[i] == Color.RED):
    FirstColorScan[i] = 0
  if (FirstColorScan[i] == Color.GREEN):
    FirstColorScan[i] = 1
  if (FirstColorScan[i] == Color.BLUE):
    FirstColorScan[i] = 2
  if (FirstColorScan[i] == Color.BLACK or FirstColorScan[i] == Color.YELLOW or FirstColorScan[i] == Color.WHITE or FirstColorScan[i] == Color.BROWN or FirstColorScan[i] == None):
    FirstColorScan[i] = 7

  if (SecColorScan[i] == Color.RED):
    SecColorScan[i] = 0
  if (SecColorScan[i] == Color.GREEN):
    SecColorScan[i] = 1
  if (SecColorScan[i] == Color.BLUE):
    SecColorScan[i] = 2
  if (SecColorScan[i] == Color.BLACK or SecColorScan[i] == Color.YELLOW or SecColorScan[i] == Color.WHITE or SecColorScan[i] == Color.BROWN or SecColorScan[i] == None):
    SecColorScan[i] = 7

for i in range(5):
  if (FirstColorScan[i] != 7):
    ColorScan[i] = FirstColorScan[i]
  else:
    ColorScan[i] = SecColorScan[i]

ColorScan[5] = LastCar(ColorScan)

file = open("color.txt", "w")
file.write("FirstColorScan = " + repr(FirstColorScan) + "\n" + "SecColorScan = " + repr(SecColorScan) + "\n" + "ColorScan = " + repr(ColorScan) + "\n")
file.close()

print(ColorScan)

robot.straight(-250)
robot.turn(-90)
robot.straight(50)
LineSquaring(1)
LeftArm.run_time(200, 400) # Open Arm
RightArm.run_time(-200, 400) # Open Arm

robot.straight(-215)
robot.turn(80)
LineSquaring(-1)
robot.turn(-20)
robot.straight(125)
robot.turn(-10)
robot.straight(200)

LeftArm.run_target(-200, 0) # Close Left Arm (Can't Close Right Arm Until Turn)
robot.turn(-55)
RightArm.run_target(200, 0) # Close Arm

robot.straight(200)
robot.turn(90)

robot.drive(-200, 25)
time.sleep(1.5)

while LeftColor.color() != Color.WHITE:
  robot.drive(-200, 25)

MotorHold()

LineFollowingToBlack('Left', 2)

'''
LineFollowingToBlack('Right', 1)
robot.straight(-150)
robot.turn(90)

LineFollowingToBlack('Right', 1)
robot.straight(-100)


if UltraSonic.distance() > 100:
  robot.turn(-90)

while RightColor.color() != Color.WHITE:
  robot.drive(-50, 0)
'''

print('Time: ' + str(time.time() - StartTime))
