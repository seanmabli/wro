#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time
import sys

def lastCar(lst):
  var = 0
  for i in lst:
    var+= lst[i]
  out = 6 - var

def MotorHold():
  robot.stop()
  left_motor.hold()
  right_motor.hold()


def LineSquaring(Num):
  THRESHOLD = (9 + 70) / 2
  # Move Forward To White
  while left_color.color() != Color.WHITE or right_color.color() != Color.WHITE:
    robot.drive(-50 * Num, 0)

  MotorHold()

  # If Left Color Detected The White
  if(left_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while left_color.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    MotorHold()
      
  # If Right Color Detected The White
  if(right_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while right_color.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    MotorHold()

  # Move Left Or Right To Get Color Reflection Equal
  while right_color.reflection() > left_color.reflection():
    right_motor.run(-10 * Num)
    left_motor.run(10 * Num)
  MotorHold()
  while right_color.reflection() < left_color.reflection():
    right_motor.run(10 * Num)
    left_motor.run(-10 * Num)
  MotorHold()

Ed = EV3Brick()

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S2)
side_color = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=139)
robot.settings(straight_speed=200, turn_rate=65)

color = []

right_motor.run_target(400, -300)
left_motor.run_target(400, -300)
while left_color.color() != Color.WHITE or right_color.color() != Color.WHITE:
  robot.drive(-50, 0)
robot.straight(-240)
robot.turn(-90)
robot.straight(150)
LineSquaring(-1)
robot.straight(150)
color.append(side_color.color())
robot.straight(115)
color.append(side_color.color())
robot.straight(115)
color.append(side_color.color())
robot.straight(115)
color.append(side_color.color())
robot.straight(115)
color.append(side_color.color())
print(color)
robot.straight(-550)
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