#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
import time, math, _thread

RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=177.8)
robot.settings(straight_speed=300)

ev3 = EV3Brick()
ev3.screen.clear()

starttime = time.time()

leftinfo = []
rightinfo = []

robot.drive(0, 100)
while time.time() - starttime < 2.5:
  leftinfo.append([LeftColor.reflection(), LeftMotor.angle()])
  rightinfo.append([RightColor.reflection(), RightMotor.angle()])
robot.stop()

compstarttime = time.time()

leftreflection = []
rightreflection = []
leftdistance = []
rightdistance = []

for i in leftinfo:
  leftreflection.append(i[0])
  leftdistance.append(abs(i[1]))
for i in rightinfo:
  rightreflection.append(i[0])
  rightdistance.append(abs(i[1]))

diffdistance = leftdistance[leftreflection.index(min(leftreflection))] - rightdistance[rightreflection.index(min(rightreflection))]
print(diffdistance)
print(math.atan(diffdistance / 65.0875) * 180 / math.pi)