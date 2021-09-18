#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time

StartTime = time.time()

# Define Variables
LocationColor = [[0, 0, 1, 2], [1, 0, 1, 2], [2, 2, 0, 1]]
LocationOccupied = [[0] * 4 for _ in range(3)]
DropoffLocation = [[0] * 4 for _ in range(3)]

FirstColorScan = [0] * 5
SecColorScan = [0] * 5
ColorScan = [0] * 5

# Define Motor & Sensor
LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)
LeftArm = Motor(Port.D)
RightArm = Motor(Port.A)

LeftColor = ColorSensor(Port.S3)
RightColor = ColorSensor(Port.S2)
SideColor = ColorSensor(Port.S1)
Ultrasonic = UltrasonicSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=55.5, axle_track=139)
robot.settings(straight_speed=200, turn_rate=65)

# Define Functions
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

def LineFollowingToBlack(Sensor, ProportionalGain):
  Threshold = (9 + 70) / 2 # Black = 9, White = 70
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    if(Sensor == 'Left'):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(-175, turn_rate)

  MotorHold()

def SetRoute(RunNum, CarColor):
  for x in range(3):
    for y in range(4):
      DropoffLocation[x][y] = 0
  CarInBay = [0 if RunNum == 1 else 3, 3 if RunNum == 1 else 5]
  for k in range(CarInBay[0], CarInBay[1]):
    for j in range(4):
      for i in range(RunNum, RunNum + 2):
        if DropoffLocation[i][j] == 0 and LocationColor[i][j] == CarColor[k] and LocationOccupied[i][j] == 0:
          DropoffLocation[i][j] = 1
          break
      else:
        continue
      break
  print(DropoffLocation)

def ArmControl(Bay):
  if Bay == 0: # All
    LeftArm.run_time(200, 400)
    RightArm.run_time(-200, 400)

  elif Bay == 1: # Bay 1
    LeftArm.run_time(200, 400)
    RightArm.run_target(200, 10)

  elif Bay == 2: # Bay 2
    LeftArm.run_target(200, 35)
    RightArm.run_target(200, -25)

  elif Bay == 3: # Bay 3
    RightArm.run_time(-200, 400)

  elif Bay == 4: # Close
    LeftArm.run_target(200, 0)
    RightArm.run_target(200, 0)

  elif Bay == 5: # Left Arm
    LeftArm.run_time(200, 400)

  elif Bay == 6: # Right Arm
    RightArm.run_time(-200, 400)

def IntoBay(CarColor, RunNum):
  robot.straight(-50)
  if CarColor == ColorScan[0 + (3 - (RunNum * 3))]:
    pass
  elif CarColor == ColorScan[1 + (3 - (RunNum * 3))]:
    robot.straight(-20)
  elif CarColor == ColorScan[2 + (3 - (RunNum * 3))]:
    robot.straight(-40)
    
  robot.turn(-80)
  robot.straight(180)

  if CarColor == ColorScan[0 + (3 - (RunNum * 3))]:
    ArmControl(1)
    ColorScan[0] = 7
  elif CarColor == ColorScan[1 + (3 - (RunNum * 3))]:
    ArmControl(2)
    ColorScan[1] = 7
  elif CarColor == ColorScan[2 + (3 - (RunNum * 3))]:
    ArmControl(3)
    ColorScan[2] = 7

  robot.straight(-180)
  ArmControl(4)
  robot.turn(80)

def Dropoff(RunNum):
  # Set And Print Directions
  SetRoute(RunNum, ColorScan)

  # RunNum 0: Top Row, RunNum 1: Middle Row
  for i in range (3):
    if DropoffLocation[RunNum][i] == 1:
      robot.straight(-30)
      if Ultrasonic.distance() > 200:
        IntoBay(LocationColor[RunNum][i], RunNum)
        DropoffLocation[RunNum][i] = 0
        LocationOccupied[RunNum][i] = 1
      else:
        LocationOccupied[RunNum][i] = 1
        SetRoute(RunNum, ColorScan) # Update Route To Avoid Obstacle
      
    if sum(DropoffLocation[RunNum][:]) == 0 and sum(DropoffLocation[RunNum + 1][i + 1 :]) == 0:
      break

    if i != 2:
      robot.straight(-50)
      LineFollowingToBlack('Left', 1)

  # Add for Colum 3
  if DropoffLocation[RunNum][3] == 1 or DropoffLocation[RunNum + 1][3] == 1:
    LineFollowingToBlack('Left', 2)
    if DropoffLocation[RunNum][3] == 1:
      robot.straight(-30)
      if Ultrasonic.distance() > 200:
        IntoBay(LocationColor[RunNum][3], RunNum)
        DropoffLocation[RunNum][3] = 0
        LocationOccupied[RunNum][3] = 1
        robot.turn(165)
        LineFollowingToBlack('Left', 1)
      else:
        LocationOccupied[RunNum][3] = 1
        SetRoute(RunNum, ColorScan) # Update Route To Avoid Obstacle

    if DropoffLocation[RunNum + 1][3] == 1:
      robot.turn(165)
      robot.straight(175)
      if Ultrasonic.distance() > 200:
        IntoBay(LocationColor[RunNum + 1][3], RunNum)
        DropoffLocation[RunNum + 1][3] = 0
        LocationOccupied[RunNum + 1][3] = 1
      else:
        LocationOccupied[RunNum + 1][3] = 1
        SetRoute(RunNum, ColorScan) # Update Route To Avoid Obstacle
      LineFollowingToBlack('Left', 1)

  else: # Turn To Next Row Down
    robot.turn(165)
    robot.straight(250)
    LineFollowingToBlack('Left', 1)

  # RunNum 0: Middle Row, RunNum 1: Bottem Row
  for j in reversed(range(i + 1)):
    if DropoffLocation[RunNum + 1][j] == 1:
      robot.straight(-30)
      if Ultrasonic.distance() > 200:
        IntoBay(LocationColor[RunNum + 1][j], RunNum)
        DropoffLocation[RunNum + 1][j] = 0
        LocationOccupied[RunNum][i] = 1
      else:
        LocationOccupied[RunNum][i] = 1
        SetRoute(RunNum, ColorScan) # Update Route To Avoid Obstacle

    if j != 0:
      robot.straight(-50)
      LineFollowingToBlack('Left', 1)

# S-Turn Out Of Base
RightMotor.run_target(400, -300)
LeftMotor.run_target(400, -300)

# Drive To First Line
while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
  robot.drive(-200, 0)

# Aline To Scan Car Color
robot.straight(-260)
robot.turn(-80)
robot.straight(150)
LineSquaring(-1)
robot.turn(2) # Squaring always is angleded to the left so this should counter that
robot.straight(35)

# First Scan
for i in range(5):
  robot.straight(115)
  FirstColorScan[i] = SideColor.color()

# Second Scan
for i in reversed(range(5)):
  SecColorScan[i] = SideColor.color()
  robot.straight(-115)

for i in range(5):
  if (FirstColorScan[i] == Color.RED):
    FirstColorScan[i] = 0
  elif (FirstColorScan[i] == Color.GREEN):
    FirstColorScan[i] = 1
  elif (FirstColorScan[i] == Color.BLUE):
    FirstColorScan[i] = 2
  else:
    FirstColorScan[i] = 7

  if (SecColorScan[i] == Color.RED):
    SecColorScan[i] = 0
  elif (SecColorScan[i] == Color.GREEN):
    SecColorScan[i] = 1
  elif (SecColorScan[i] == Color.BLUE):
    SecColorScan[i] = 2
  else:
    SecColorScan[i] = 7

  ColorScan[i] = FirstColorScan[i] if FirstColorScan[i] != 7 else SecColorScan[i]

# If Connected Print To Terminal
print(ColorScan)

# Move To Pickup Run 1
robot.straight(-250)
robot.turn(-90)
robot.straight(50)
LineSquaring(1)

ArmControl(0) # Open Arms
robot.straight(-190)
robot.turn(80)
LineSquaring(-1)

# Pickup Run 1
robot.turn(-15)
robot.straight(125)
robot.turn(-10)
robot.straight(100)
robot.turn(10)
robot.straight(20)

ArmControl(4) # Close Arms

# Move To Dropoff 1
robot.turn(15)
robot.straight(-325)
robot.turn (80)
LineFollowingToBlack('Left', 1)

Dropoff(1) # Dropoff Run 1

# Second Pickup
ArmControl(0) # Open Arms To Let Remaining Cars Drop and Prep for pickup

robot.straight(-50)
Threshold = (9 + 70) / 2 # Black = 9, White = 70
while LeftColor.reflection() > 15:
  Deviation = - (LeftColor.reflection() - Threshold)
  robot.drive(-200, Deviation)

MotorHold()
robot.straight(-170)

robot.turn(90)
LineSquaring(-1)
robot.turn(4) # Squaring always is angleded to the left so this should counter that

robot.straight(350) # First Car In Bay
robot.turn(-20)
robot.straight(120) # Second Car In Bay

ArmControl(4) # Close Arms

robot.straight(-75)
robot.turn(100)
robot.straight(-300)

LineFollowingToBlack('Left', 1)

Dropoff(0)

print('Time: ' + str(time.time() - StartTime))