#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
import time
# import numpy as np

def LastCar(CarColor):
  return 6 - np.sum(CarColor) # 1 + 1 + 2 + 2 = 6

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

def LineFollowing(Distance, Sensor):
  # Black = 9, White = 70
  # Circumfrence = 34.56
  threshold = (9 + 70) / 2

  PROPORTIONAL_GAIN = 4
  LeftMotor.reset_angle(0)
  RightMotor.reset_angle(0)
  while (LeftMotor.angle() + RightMotor.angle()) / 2 < (Distance / 174.36) * 360:
    if(Sensor == 'Left'):
      deviation = (LeftColor.reflection() - threshold)
    if(Sensor == 'Right'):
      deviation = (RightColor.reflection() - threshold)
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(-100, turn_rate)

  MotorHold()

LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)
LeftArm = Motor(Port.D)
RightArm = Motor(Port.A)

LeftColor = ColorSensor(Port.S3)
RightColor = ColorSensor(Port.S2)
SideColor = ColorSensor(Port.S1)
robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=55.5, axle_track=139)
robot.settings(straight_speed=200, turn_rate=65)

# FirstColorScan = numpy.zeros(6)
# SecColorScan = numpy.zeros(6)

RightMotor.run_target(400, -300)
LeftMotor.run_target(400, -300)
while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
  robot.drive(-50, 0)
robot.straight(-240)
robot.turn(-90)
robot.straight(150)
LineSquaring(-1)
robot.straight(150)
# FirstColorScan[0] = SideColor.color()
robot.straight(115)
# FirstColorScan[1] = SideColor.color()
robot.straight(115)
# FirstColorScan[2] = SideColor.color()
robot.straight(115)
# FirstColorScan[3] = SideColor.color()
robot.straight(115)
# FirstColorScan[4] = SideColor.color()

robot.turn(-5)
robot.turn(5)

# SecColorScan[4] = SideColor.color()
robot.straight(-115)
#SecColorScan[3] = SideColor.color()
robot.straight(-115)
# SecColorScan[2] = SideColor.color()
robot.straight(-115)
# SecColorScan[1] = SideColor.color()
robot.straight(-115)
# SecColorScan[0] = SideColor.color()
robot.straight(-115)

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

ColorScan = np.where(FirstColorScan == Color.RED, 0)
ColorScan = np.where(FirstColorScan == Color.GREEN, 1)
ColorScan = np.where(FirstColorScan == Color.BLUE, 2)

ColorScan = np.where(FirstColorScan == Color.BLACK, 7) # Replace With None
ColorScan = np.where(FirstColorScan == Color.YELLOW, 7) # Replace With None
ColorScan = np.where(FirstColorScan == Color.WHITE, 7) # Replace With None
ColorScan = np.where(FirstColorScan == Color.BROWN, 7) # Replace With None
ColorScan = np.where(FirstColorScan == None, 7)

ColorScan = np.where(ColorScan == 7, SecColorScan)

print(ColorScan)
'''
countdown(30)
LineSquaring(1)
robot.straight(-200)
robot.turn(-90)
robot.straight(50)
LineSquaring(1)
LeftArm.run_time(200, 400) # Open Arm
RightArm.run_time(-200, 400) # Open Arm
LeftArm.brake()
RightArm.brake()
robot.straight(-220)
robot.turn(75)
LineSquaring(-1)
robot.turn(-20)
robot.straight(125)
robot.turn(-10)
robot.straight(250)

LeftArm.run_target(-200, 55) # Close Arm
RightArm.run_target(200, 55) # Close Arm
robot.turn(-55)