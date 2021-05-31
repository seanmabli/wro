#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import time
import Functions as F

Ed = EV3Brick()

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
left_color = ColorSensor(Port.S3)
right_color = ColorSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=120)
robot.settings(straight_speed=400, turn_rate=65)

F.LineSquaring()