#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time

GrabMotor = Motor(Port.A)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
LiftMotor = Motor(Port.D)

Gyro = GyroSensor(Port.S1)
LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
FrontColor = ColorSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings( straight_speed=300)

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
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  
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

def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, kp=0.25, ki=0, kd=0.5, speed=160): # wait distance is the # of mm after a black it waits until continue detecting blacks
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')

  if sensor == LeftColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

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

    robot.drive(speed, turn)
    gyrodev.append(abs(Gyro.angle() - lastgyro))
    lastgyro = Gyro.angle()
  robot.stop()
  
def gurn(turn, tp='tank', fb='forward', speed=100): # gurn = gyro + turn ;)
  if tp not in ['tank', 'pivot']:
    raise Exception('tp must be "tank" or "pivot"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if fb == 'backward':
    speed *= -1

  startangle = robot.angle()
  if tp == 'tank':
    if turn < 0:
      robot.drive(0, speed)
    else:
      robot.drive(0, -speed)
  elif tp == 'pivot':
    if turn < 0:
      LeftMotor.run(speed)
    else:
      RightMotor.run(speed)

  while abs(startangle - robot.angle()) < abs(turn):
    pass

  if tp == 'tank':
    robot.stop()
  elif tp == 'pivot':
    LeftMotor.hold()
    RightMotor.hold()

def sTurn(rl, fb, turn, tp='pivot', drive=0, turn_rate=100): # rl = right-left, fb = forward-backward, turn = turn degrees(posotive), drive = drive between turns(positive)
  if rl not in ['right', 'left']:
    raise Exception('rl must be "right" or "left"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if rl == 'right':
    turn *= -1
  if fb == 'backward':
    drive *= -1

  gurn(turn, tp=tp, fb=fb, speed=turn_rate)
  if drive != 0:
    robot.straight(drive)
  robot.stop()
  gurn(-turn, tp=tp, fb=fb, speed=turn_rate)

def straight(distance):
  robot.straight(distance)
  robot.stop()

def sweep(sensor):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')

  pass

def grab(oc='open'):
  if oc == 'open':
    GrabMotor.stop()
    time.sleep(0.1)
    GrabMotor.run_angle(100, -30)
  elif oc == 'close':
    GrabMotor.run(200)

def lift(up='up'):
  if up == 'up':
    LiftMotor.run_angle(200, 80)
  elif up == 'down':
    LiftMotor.run_angle(200, -80)

ev3 = EV3Brick()
ev3.screen.clear()
startangle = Gyro.angle()
while Button.CENTER not in ev3.buttons.pressed():
  pass
print(Gyro.angle() - startangle)

starttime = time.time()

'''
# old start code
straight(70)
lfpidBlack()
gurn(-90)
sTurn(rl='left', fb='forward', tp='pivot', turn=35, turn_rate=200)
lfpidBlack(sensor=LeftColor, sideofsensor='out')
lift('up')
sTurn(rl='left', fb='backward', tp='tank', drive=150, turn=90, turn_rate=60)
straight(-260)
gurn(20)
lift('down')
gurn(-20)
sTurn(rl='right', fb='forward', tp='tank', drive=110, turn=45, turn_rate=60)
lfpidBlack(sensor=LeftColor, sideofsensor='out')
gurn(-90, tp='pivot', speed=300)
lfpidBlack(sensor=LeftColor, sideofsensor='out')
gurn(90, tp='pivot', speed=300)
straight(360)
gurn(-90, tp='pivot', speed=300)
'''

straight(70)
lfpidBlack(sensor=LeftColor, sideofsensor='in')
straight(100)
gurn(-90)


print(time.time() - starttime)