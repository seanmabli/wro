#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase

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

Ed = EV3Brick()

LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)
LeftColor = ColorSensor(Port.S3)
RightColor = ColorSensor(Port.S2)
SideColor = ColorSensor(Port.S1)
robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=55.5, axle_track=139)
robot.settings(straight_speed=200, turn_rate=65)

FirstColorScan = []
SecColorScan = []

RightMotor.run_target(400, -300)
LeftMotor.run_target(400, -300)
while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
  robot.drive(-50, 0)
robot.straight(-240)
robot.turn(-90)
robot.straight(150)
LineSquaring(-1)
robot.straight(150)
FirstColorScan.append(SideColor.color())
robot.straight(115)
FirstColorScan.append(SideColor.color())
robot.straight(115)
FirstColorScan.append(SideColor.color())
robot.straight(115)
FirstColorScan.append(SideColor.color())
robot.straight(115)
FirstColorScan.append(SideColor.color())

robot.turn(-5)
robot.turn(5)

SecColorScan.append(SideColor.color())
robot.straight(-115)
SecColorScan.append(SideColor.color())
robot.straight(-115)
SecColorScan.append(SideColor.color())
robot.straight(-115)
SecColorScan.append(SideColor.color())
robot.straight(-115)
SecColorScan.append(SideColor.color())
robot.straight(-115)
SecColorScan.append(SideColor.color())
SecColorScan = SecColorScan.reverse() 

print(FirstColorScan)
print(SecColorScan)

LineSquaring(1)
robot.straight(-200)
robot.turn(-90)
robot.straight(50)
LineSquaring(1)
robot.straight(-220)
robot.turn(75)
LineSquaring(-1)
robot.turn(-20)
robot.straight(125)
robot.turn(-10)
robot.straight(125)
robot.turn(-55)