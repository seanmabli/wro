#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
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

BLACK = 9
WHITE = 70
threshold = (BLACK + WHITE) / 2

PROPORTIONAL_GAIN = 4

while True:
    deviation = ((left_color.reflection() - threshold) + (right_color.reflection() - threshold)) / 2
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(100, turn_rate)

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