#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
import time
import multiprocessing

GrabMotor = Motor(Port.A)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
LiftMotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)

Gyro = GyroSensor(Port.S1)
LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
FrontColor = ColorSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=300)

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

def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, kp=0.25, ki=0, kd=0.5, speed=160, threshold='x'): # wait distance is the # of mm after a black it waits until continue detecting blacks
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  if threshold not in ['t', 'x'] and not threshold.isint():
    raise Exception('threshold must be "t" or "x" or an integer')

  if threshold == 't':
    threshold = 22
  elif threshold == 'x':
    threshold = 15

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
    if (LeftColor.reflection() + RightColor.reflection()) / 2 < threshold and lastdistance + waitdistance < abs(robot.distance()):
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
  
def gurn(turn, aggresion=30, tp='tank', fb='forward', speed=100): # gurn = gyro + turn ;)
  if tp not in ['tank', 'pivot', 'circle']:
    raise Exception('tp must be "tank" or "pivot" or "circle"')
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
  elif tp == 'circle':
    if turn < 0:
      robot.drive(speed, -aggresion)
    else:
      robot.drive(speed, aggresion)

  while abs(startangle - robot.angle()) < abs(turn):
    pass

  if tp == 'tank':
    robot.stop()
  elif tp == 'pivot':
    LeftMotor.hold()
    RightMotor.hold()

def fraudulo(dividend, divisor): # dividend / divisor
  signa = 1 if dividend > 0 else -1
  signb = 1 if divisor > 0 else -1
  dividend, divisor = abs(dividend), abs(divisor)
  return signa * signb * (dividend - divisor * (dividend // divisor)) / divisor

def orient(speed=30):
  print(Gyro.angle(), startangle)

  '''
  if Gyro.angle() < startangle:
    if Gyro.angle() % 360 > 180:
      robot.drive(0, -speed)
    elif Gyro.angle() % 360  180:
    robot.drive(0, speed)

  '''
  if Gyro.angle() < 0:
    while fraudulo(Gyro.angle(), 360) < 0:
      robot.drive(0, -speed)
      print(Gyro.angle())
  else:
    while mod(Gyro.angle(), 360) > 0:
      robot.drive(0, speed)
      print(Gyro.angle())

  robot.stop()

def colorScan(sensor, threshold):
  if threshold[0] < sensor.reflection() < threshold[1]:
    return sensor.color()


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
    GrabMotor.run_angle(100, 50)
  elif oc == 'close':
    GrabMotor.run(-200)
    time.sleep(0.6)

def lift(ud='up'):
  if ud == 'up':
    LiftMotor.run_angle(400, -320)
  elif ud == 'down':
    LiftMotor.run_angle(400, 320)
  elif ud == "downhalf":
    LiftMotor.run_angle(400, 160)
  elif ud == "downfull":
    LiftMotor.run_angle(400, 550)

ev3 = EV3Brick()
ev3.screen.clear()
startangle = Gyro.angle()
while Button.CENTER not in ev3.buttons.pressed():
  pass
print(Gyro.angle() - startangle)

Gyro.reset_angle(0)
starttime = time.time()

baystatus = []

# Pickup laundry old
'''
straight(70)
lfpidBlack(sensor=RightColor, sideofsensor='in')
straight(120)
gurn(90)
lfpidBlack(sensor=RightColor, sideofsensor='in', threshold='t')   
straight(-20)
sTurn(rl='left', fb='forward', turn=60, turn_rate=250)
gurn(-90, tp='pivot', speed=200)
grab(oc='open')
straight(-150) # maybe change to gyro stright
LiftMotor.hold()
grab(oc='close')
time.sleep(0.5)
grab(oc='open')
straight(-100)
grab(oc='close')
straight(220)
grab(oc='open')
straight(-100)
grab(oc='close')
baystatus.append({"type": "water"})
baystatus.append({"type": "water"})
gurn(60, aggresion=45, tp='circle', speed=200)
straight(220)
gurn(-30, tp="pivot", speed=200)
robot.stop()
'''

# Pickup laundry new
gurn(15, tp='pivot', speed=200)
straight(465)
gurn(-58, tp='pivot', speed=200)
straight(-70)
grab(oc="open")
lift(ud="downfull")
grab(oc='close')
grab(oc='open')
straight(-100)
grab(oc='close')
straight(210)
grab(oc='open')
straight(-100)
grab(oc='close')
baystatus.append({"type": "water"})
baystatus.append({"type": "water"})
gurn(60, aggresion=45, tp='circle', speed=200)
straight(220)
gurn(-30, tp="pivot", speed=200)
robot.stop()
straight(15)
print(FrontColor.color()) # marking block

# Red box
straight(15)
gurn(-90, fb="backward", tp="pivot", speed=200)
print(FrontColor.color()) # laundry block
straight(315)
lift(ud="up")
'''
gurn(-45, fb="forward", tp="pivot", speed=200)
straight(-150)
gurn(35, fb="backward", tp="pivot", speed=200)
grab(oc="open")
straight(-50)
lift(ud="down")
grab(oc="close")
lift(ud="up")
gurn(90, fb="backward", tp="pivot", speed=200)
straight(-70)
lift(ud="downhalf")
grab(oc="open")
'''

print(time.time() - starttime)