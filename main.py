#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time

Ed = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
left_color = ColorSensor(Port.S2)
right_color = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=120)
robot.settings(straight_speed=400, turn_rate=65)

def LineFollowing(DistanceCM, Sensor):
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

def LineSquaring():
  while left_color.color() != Color.WHITE:
    robot.drive(100, 0)

def Metric(inches):
  return inches * 25.4 

LineFollowing(100, 'Left')

'''
robot.straight(600)
robot.turn(-90)
robot.straight(900)
robot.straight(-700)
robot.turn(90)
robot.straight(1500)
robot.straight(-2100)

Ed.screen.clear()
Ed.screen.draw_text(0, 0, (time.time() - StartTime))
time.sleep(10)
'''