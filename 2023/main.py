#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
import time
import _thread

BoatMotor = Motor(Port.A)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
LeftMotor = Motor(Port.C)
ArmMotor = Motor(Port.D)

ColorA = ColorSensor(Port.S1) # Name TBD
LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
# ColorB = ColorSensor(Port.S4) # Name TBD

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=700)

def main():
  # straight(100)
  # lfpidBlack(sensor=LeftColor, sideofsensor='in', startdistance=100, threshold=10)
  while True:
    print(LeftColor.reflection(), RightColor.reflection())

def straight(distance):
  robot.straight(distance)
  robot.stop()

def square(threshold, speed):
  leftBlack = False
  rigthtBlack = False
  if not leftBlack and not rigthtBlack:
    RightMotor.run(speed)
    LeftMotor.run(speed)
  while not leftBlack and not rigthtBlack:
    if LeftColor.reflection() <= threshold:
      leftBlack = True
      LeftMotor.hold()
    elif RightColor.reflection() <= threshold:
      rigthtBlack = True
      RightMotor.hold()
  
  if not rigthtBlack:
    RightMotor.run(speed)
  while not rigthtBlack:
    if RightColor.reflection() <= threshold:
      rigthtBlack = True
      RightMotor.hold()
  
  if not leftBlack:
    LeftMotor.run(speed)
  while not leftBlack:
    if LeftColor.reflection() <= threshold:
      leftBlack = True
      LeftMotor.hold()

def durn(turn, aggresion=30, tp='tank', fb='forward', speed=100): # gurn = degree turn
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

  if tp == 'tank' or tp == 'circle':
    robot.stop()
  elif tp == 'pivot':
    LeftMotor.hold()
    RightMotor.hold()

def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, startdistance=0, kp=0.25, ki=0, kd=0.5, startncap=[], estdistance=0, threshold='x',): # wait distance is the # of mm after a black it waits until continue detecting blacks
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  if threshold not in ['t', 'x'] and not isinstance(threshold, int) and isinstance(threshold, float):
    raise Exception('threshold must be "t" or "x" or an integer')

  if threshold == 't': # 3 way intercetion
    threshold = 22
  elif threshold == 'x': # 4 way intercetion
    threshold = 15

  if sensor == LeftColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if startncap == []:
    speed = [160]
  elif type(startncap) == int:
    speed = [startncap]
  else:
    speed = list(range(startncap[0], startncap[1], 1 if startncap[0] < startncap[1] else -1))
  
  target = (8 + 76) / 2 # Black  = 8, White = 76
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0
  count = 0

  robot.reset()
  lastdistance = abs(robot.distance())
  lastdistancechange = 0
  num = 0
  while count < blacks:
    if (LeftColor.reflection() + RightColor.reflection()) / 2 < threshold and lastdistance + waitdistance < abs(robot.distance()) and startdistance < abs(robot.distance()):
      count += 1
      lastdistance = abs(robot.distance())
      print((LeftColor.reflection() + RightColor.reflection()) / 2)

    if abs(lastdistancechange - robot.distance()) > estdistance / len(speed) and num < len(speed) - 1:
      lastdistancechange = robot.distance()
      num += 1

    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    robot.drive(speed[num], turn)

  robot.stop()

def lfpidDistance(distance, sensor=RightColor, sideofsensor='in', startncap=[], kp=0.25, ki=0, kd=0.5):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')
  
  if sensor == LeftColor:
    sideofsensor = 'in' if sideofsensor == 'out' else 'out'

  if startncap == []:
    speed = [160]
    one = True
  elif type(startncap) == int:
    speed = [startncap]
    one = True
  else:
    speed = list(range(startncap[0], startncap[1]))
    one = False

  target = (8 + 76) / 2 # Black  = 8, White = 76
  gyrodev = []
  error = 0
  lasterror = 0
  integral = 0
  derivative = 0

  robot.reset()

  lastdistance = 0
  num = 0
  while abs(robot.distance()) < distance:
    if abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if sideofsensor == 'out':
      error = target - sensor.reflection()
    elif sideofsensor == 'in':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    if one:
      robot.drive(speed[0], turn)
    else:
      robot.drive(speed[num], turn)

  robot.stop()

def boatGrab(oc='open', percentage=1, pinch=True):
  if oc == 'open':
    BoatMotor.stop()
    time.sleep(0.1)
    BoatMotor.run_angle(400, 200 * percentage)
  elif oc == 'close':
    BoatMotor.run(-200)
    time.sleep(0.6 * percentage)
    if not pinch:
      BoatMotor.stop()

def armGrab(oc='open'):
  if oc == 'open':
    time.sleep(0.001)
    ArmMotor.run_angle(400, -50)
  elif oc == 'close':
    ArmMotor.run(200)
    time.sleep(1)
    ArmMotor.stop()

starttime = time.time()
main()
print(time.time() - starttime)