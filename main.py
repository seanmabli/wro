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

FirstColorScan = [0] * 6
SecColorScan = [0] * 6
ColorScan = [0] * 6

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
    robot.drive(-100, turn_rate)

  MotorHold()

def LastCar(CarColor):
  return 6 - sum(CarColor) # 0 + 0 + 1 + 1 + 2 + 2 = 6

def SetLocation(RunNum, CarColor):
  for k in range(3):
    for j in range(4):
      for i in range(2):
        if DropoffLocation[i + RunNum][j] == 0 and LocationColor[i + RunNum][j] == CarColor[k] and LocationOccupied[i + RunNum][j] == 0:
          DropoffLocation[i + RunNum][j] = 1
          break
      else:
        continue
      break

def ArmControl(Bay):
  if Bay == 0: # All
    LeftArm.run_time(200, 400)
    RightArm.run_time(-200, 400)

  elif Bay == 1: # Bay 1
    LeftArm.run_time(200, 400)
    RightArm.run_target(200, 10)

  elif Bay == 2: # Bay 2
    LeftArm.run_target(200, 35)
    RightArm.run_target(200, -30)

  elif Bay == 3: # Bay 3
    RightArm.run_time(200, -400)

  elif Bay == 4: # Close
    LeftArm.run_target(200, 0)
    RightArm.run_target(200, 0)

  elif Bay == 5: # Left Arm
    LeftArm.run_time(200, 400)

  elif Bay == 6: # Right Arm
    RightArm.run_time(-200, 400)

def Dropoff():
  robot.turn(90)
  robot.turn(-90)

# S-Turn
RightMotor.run_target(400, -300)
LeftMotor.run_target(400, -300)

# Drive To First Line
while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
  robot.drive(-200, 0)

# Aline To Scan Car Color
robot.straight(-265)
robot.turn(-80)
robot.straight(150)
LineSquaring(-1)
robot.turn(4) # Squaring always is angleded to the left so this should counter that
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

  if (FirstColorScan[i] != 7):
    ColorScan[i] = FirstColorScan[i]
  else:
    ColorScan[i] = SecColorScan[i]

ColorScan[5] = LastCar(ColorScan)

# Save To Text File Because Of Posible Disconnect
file = open("color.txt", "w")
file.write("FirstColorScan = " + repr(FirstColorScan) + "\n" + "SecColorScan = " + repr(SecColorScan) + "\n" + "ColorScan = " + repr(ColorScan) + "\n")
file.close()

# If Connected Print
print(ColorScan)

robot.straight(-250)
robot.turn(-90)
robot.straight(50)
LineSquaring(1)

LeftArm.run_time(200, 400) # Open Arm
RightArm.run_time(-200, 400) # Open Arm

robot.straight(-207)
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

LineFollowingToBlack('Left', 2)

SetLocation(1, ColorScan)

# Middle Row
for i in range (4):
  if DropoffLocation[1][i] == 1:
    if Ultrasonic.distance() > 100:
      Dropoff()
      LocationOccupied[1][i] = 1
      DropoffLocation[1][i] = 0
    else:
      LocationOccupied[1][i] = 1
      SetLocation(RunNum, CarColor)

  if DropoffLocation[1][:] == [0, 0, 0, 0]:
    break
  else:
    robot.straight(-50)
    LineFollowingToBlack('Left', 1)
  
robot.straight(-300)
robot.turn(160)
LineFollowingToBlack('Left', 2)

# Bottem Row
for j in reversed(range(i)):
  if DropoffLocation[2][j] == 1:
    if Ultrasonic.distance() > 100:
      Dropoff()
      LocationOccupied[2][j] = 1
      DropoffLocation[2][j] = 0
    else:
      LocationOccupied[2][j] = 1
      SetLocation(RunNum, CarColor)
  robot.straight(-50)
  LineFollowingToBlack('Left', 1)

print('Time: ' + str(time.time() - StartTime))