#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time

LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
Gyro = GyroSensor(Port.S4)

axle_track = 157

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=axle_track)
robot.settings( straight_speed=200, turn_rate=65)

def lfBlack(Sensor, ProportionalGain=1):
  Threshold = (9 + 74) / 2 # Black = 9, White = 74
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    if(Sensor == 'Left'):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == 'Right'):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(180, turn_rate)

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
    robot.drive(180, turn_rate)

  robot.stop()

def lfpidDistance(distance, sensor=RightColor, sideofsensor='in', kp=0.25, ki=0, kd=0.5):
  target = (9 + 74) / 2 # Black = 9, White = 74
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0

  robot.reset()
  lastgyro = Gyro.angle()
  while abs(robot.distance()) < distance:
    if sideofsensor == 'in':
      error = target - sensor.reflection()
    elif sideofsensor == 'out':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    robot.drive(180, turn)
    gyrodev.append(abs(Gyro.angle() - lastgyro))
    lastgyro = Gyro.angle()

  return sum(gyrodev[100 : -100]) / (len(gyrodev) - 200)
  # return sum(gyrodev) / len(gyrodev)

def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, kp=0.25, ki=0, kd=0.5):
  target = (9 + 74) / 2 # Black = 9, White = 74
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0
  count = 0

  robot.reset()
  lastgyro = Gyro.angle()
  lastdistance = abs(robot.distance())
  while count < blacks:
    if (LeftColor.reflection() + RightColor.reflection()) / 2 < 15 and lastdistance + waitdistance < abs(robot.distance()):
      count += 1
      lastdistance = abs(robot.distance())

    if sideofsensor == 'in':
      error = target - sensor.reflection()
    elif sideofsensor == 'out':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    robot.drive(180, turn)
    gyrodev.append(abs(Gyro.angle() - lastgyro))
    lastgyro = Gyro.angle()
  robot.stop()
  
def gurn(turn, tp='tank', speed=100): # gurn = gyro + turn ;)
  startangle = robot.angle()
  if tp == 'tank':
    robot.drive(0, -speed)
  elif tp == 'pivot':
    if turn < 0:
      LeftMotor.run(speed)
    else:
      RightMotor.run(speed)
  while abs(startangle - robot.angle()) < abs(turn):
    pass
  robot.stop()
  LeftMotor.stop()
  RightMotor.stop()

def sTurn(rl, turn, fb='forward', drive=0, turn_rate=100): # rl = right-left, fb = forward-backward, turn = turn degrees(posotive), drive = drive between turns(positive)
  if rl == 'right':
    turn *= -1
  if fb == 'backward':
    drive *= -1

  gurn(turn, tp='pivot', speed=turn_rate)
  if drive != 0:
    robot.straight(drive)
  gurn(-turn, tp='pivot', speed=turn_rate)

ev3 = EV3Brick()
ev3.screen.clear()
startangle = Gyro.angle()
starttime = time.time()
while Button.CENTER not in ev3.buttons.pressed():
  pass
print(Gyro.angle() - startangle)

#print(lfpid('right', distance=1500, kd=0.4))

# 0.25, 0, 0 = 0.01955
# 0.25, 0, 0.4 = 0.01286
# 0.25, 0, 0.5 = 0.01639
# 0.25, 0, 1 = 0.017889

# gurn(90, tp='tank')

# robot.straight(70)
# lfpidBlack()
# gurn(90)
# sTurn('left', 'forward', 40, 70)
# lfpidDistance(distance=150, sensor=LeftColor, sideofsensor='out')

sTurn('left', 90, turn_rate=200)
sTurn('rigtt', 90, turn_rate=200)
sTurn('left', 90, 'backward', turn_rate=200)
sTurn('right', 90, 'backward', turn_rate=200)
print(time.time() - starttime)