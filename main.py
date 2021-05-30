#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time

Ed = EV3Brick()

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=120)
robot.settings(straight_speed=400, turn_rate=65)

def MotorHold():
  robot.stop()
  left_motor.hold()
  right_motor.hold()

def LineFollowing(Distance, Sensor):
  # Black = 9, White = 70
  # Circumfrence = 34.56
  threshold = (9 + 70) / 2

  PROPORTIONAL_GAIN = 4
  left_motor.reset_angle(0)
  right_motor.reset_angle(0)
  while (left_motor.angle() + right_motor.angle()) / 2 < (Distance / 174.36) * 360:
    if(Sensor == 'Left'):
      deviation = (left_color.reflection() - threshold)
    if(Sensor == 'Right'):
      deviation = (right_color.reflection() - threshold)
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(100, turn_rate)

  MotorHold()

def LineSquaring():
  THRESHOLD = (9 + 70) / 2
  # Move Forward To White
  while left_color.color() != Color.WHITE or right_color.color() != Color.WHITE:
    robot.drive(-100, 0)

  MotorHold()

  # If Left Color Detected The White
  if(left_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while left_color.reflection() >= THRESHOLD:
      robot.drive(-25, 0)
    MotorHold()

    # Move Left Or Right To Get Color Reflection Equal
    EqualReflection(THRESHOLD)
      
  # If Right Color Detected The White
  if(right_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while right_color.reflection() >= THRESHOLD:
      robot.drive(-25, 0)
    MotorHold()

    # Move Left Or Right To Get Color Reflection Equal
    EqualReflection(THRESHOLD)

def EqualReflection(THRESHOLD):
  while right_color.reflection() > left_color.reflection():
    right_motor.run(-10)
    left_motor.run(10)
  MotorHold()
  while right_color.reflection() < left_color.reflection():
    right_motor.run(10)
    left_motor.run(-10)
  MotorHold()
  print(right_color.reflection())
  print(left_color.reflection())

'''
  if(right_color.color() == Color.WHITE):  
    while right_color.reflection() > 41:
      robot.drive(-100, 0)
    MotorHold()
    while left_color.reflection() > 41:
      right_motor.run(-100)
    MotorHold()

  while left_color.reflection() > 41 and left_color.reflection() < 39:
    while left_color.reflection() > 41:
      left_motor.run(-100)
    while left_color.reflection() < 39:
      left_motor.run(100)
    
  while right_color.reflection() > 41 and right_color.reflection() < 39:
    while right_color.reflection() > 41:
      right_motor.run(-100)
    while right_color.reflection() < 39:
      right_motor.run(100)

  MotorHold()
    '''
LineSquaring()