#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button
from pybricks.robotics import DriveBase

LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)

LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
Gyro = GyroSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=200, turn_rate=65)

robot.straight(-260)