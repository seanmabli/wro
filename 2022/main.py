#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button
from pybricks.robotics import DriveBase
import time

LeftMotor = Motor(Port.C)
RightMotor = Motor(Port.B)

LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
Gyro = GyroSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=200, turn_rate=65)

def lfBlack(Sensor, ProportionalGain=1):
  Threshold = (9 + 74) / 2 # Black = 9, White = 74
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    if(Sensor == 'Left'):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(-180, turn_rate)

  robot.stop()

def lfDistance(Sensor, distance, ProportionalGain=1):
  Threshold = (9 + 74) / 2 # Black = 9, White = 74
  robot.reset()
  while abs(robot.distance()) < distance:
    if(Sensor == 'Left'):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(-180, turn_rate)

  robot.stop()

def lfpid(sensor, distance, kp=1, ki=0, kd=0):
  target = (9 + 74) / 2 # Black = 9, White = 74
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0

  robot.reset()
  while abs(robot.distance()) < distance:
    error = target - RightColor.reflection()
    integral += error
    derivative = error - lasterror

    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    robot.drive(-180, turn)
    gyrodev.append(abs(Gyro.angle()))

  # return sum(gyrodev[100 : -100]) / (len(gyrodev) - 200)
  return sum(gyrodev) / len(gyrodev)


x = Gyro.angle()
time.sleep(2)
print(x - Gyro.angle())

print(lfpid('right', distance=1500 , kp=0.25))
# 0.1 = 1.44
# 1 = 2.8
# 5 = 25
# 0.3 = 0.57
# 0.25 = 0.51