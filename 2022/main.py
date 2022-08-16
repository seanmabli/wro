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
    if(Sensor == LeftColor):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == RightColor):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(180, turn_rate)

  robot.stop()

def lfDistance(sensor, distance, sideofsensor, proportionalGain=0.5):
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')

  threshold = (9 + 74) / 2 # Black = 9, White = 74
  sideofsensor = -1 if sideofsensor == 'in' else 1
  robot.reset()

  while abs(robot.distance()) < distance:
    if(sensor == LeftColor):
      Deviation = - (LeftColor.reflection() - threshold) * sideofsensor
    if(sensor == RightColor):
      Deviation = (RightColor.reflection() - threshold) * sideofsensor
    turn_rate = proportionalGain * Deviation
    robot.drive(150, turn_rate)

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

  if tp == 'tank' or tp == 'circle':
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

def sweep(sensor, direction, speed=100):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if direction not in ['right', 'left']:
    raise Exception('direction must be "right" or "left"')

  if direction == 'right':
    robot.drive(0, speed)
  else:
    robot.drive(0, -speed)
  
  while sensor.reflection() > 15:
    pass

  robot.stop()
  
  if direction == 'right':
    gurn(6, tp='tank', fb='forward', speed=(speed / 2))
  else:
    gurn(-6, tp='tank', fb='forward', speed=(speed / 2))

def grab(oc='open'):
  if oc == 'open':
    GrabMotor.stop()
    time.sleep(0.1)
    GrabMotor.run_angle(400, 200)
  elif oc == 'close':
    GrabMotor.run(-400)
    time.sleep(0.6)

def lift(ud='up'):
  if ud == 'up':
    LiftMotor.run_angle(400, -320)
  elif ud == 'uphalf':
    LiftMotor.run_angle(400, -160)
  elif ud == 'down':
    LiftMotor.run_angle(400, 320)
  elif ud == "downhalf":
    LiftMotor.run_angle(400, 160)
  elif ud == "downfull":
    LiftMotor.run_angle(400, 550)

def colorScan(acceptable, direction):
  ev3.light.on(Color.RED)
  if FrontColor.color() in acceptable:
    return FrontColor.color()
  else:
    Gyro.reset_angle(0)
    if direction == 'in':
      RightMotor.run(-150)
    elif direction == 'out':
      RightMotor.run(150)

    while FrontColor.color() not in acceptable and abs(Gyro.angle()) < 40:
      pass

    robot.stop()

    color = FrontColor.color()

    if direction == 'in':
      RightMotor.run(75)
      while Gyro.angle() < 0:
        pass
    elif direction == 'out':
      RightMotor.run(-75)
      while Gyro.angle() > 0:
        pass
    robot.stop()
    ev3.light.on(Color.GREEN)
    return color

def LineSquaring(Num):
  THRESHOLD = (9 + 70) / 2
  # Move Forward To White
  while LeftColor.color() != Color.WHITE or RightColor.color() != Color.WHITE:
    robot.drive(-50 * Num, 0)

  robot.stop()

  # If Left Color Detected The White
  if(LeftColor.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while LeftColor.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    robot.stop()
      
  # If Right Color Detected The White
  if(RightColor.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while RightColor.reflection() >= THRESHOLD:
      robot.drive(-10 * Num, 0)
    robot.stop()

  # Move Left Or Right To Get Color Reflection Equal
  while RightColor.reflection() > LeftColor.reflection():
    RightMotor.run(-10 * Num)
    LeftMotor.run(10 * Num)
  robot.stop()
  while RightColor.reflection() < LeftColor.reflection():
    RightMotor.run(10 * Num)
    LeftMotor.run(-10 * Num)
  robot.stop()

# in mm
water = 64
laundry = 32
watertowater = 64
watertolaundry = 48
laundrytolaundry = 32
grabtowater = 63
grabtolaundry = 47
grabtoback = 248

def frombay(baystatus, object, position):
  if object["type"] not in ["water", "laundry"]:
    raise Exception('object must be {"type" : "water"} or {"type" : "laundry", "color" : (Color.RED or Color.YELLOW or Color.Black)}')
  if position not in ["front", "back"]:
    raise Exception('position must be "front" or "back"')
  
  dis = 0
  for i, item in enumerate(reversed(baystatus)):
    if i == 0:
      if position == "front":
        dis = grabtowater if item["type"] == "water" else grabtolaundry
      elif position == "back":
        fulldis = 0
        for item2 in baystatus:
          fulldis += water if item2["type"] == "water" else laundry
        dis = grabtoback - fulldis + int(water / 2 if item["type"] == "water" else laundry / 2)
      else:
        raise Exception('baystatus error 0')
    elif list(reversed(baystatus))[i - 1]["type"] == "water":
      dis += watertowater if item["type"] == "water" else watertolaundry
    elif list(reversed(baystatus))[i - 1]["type"] == "laundry":
      dis += laundrytolaundry if item["type"] == "laundry" else watertolaundry
    else:
      raise Exception('baystatus error 1')

    if object == item:
      break

  print("dis:", dis)
  
  grab(oc='open')
  straight(dis)
  grab(oc='close')
  lift(ud='up')
  straight(-dis - 35)

  baystatus.remove(object)
  return baystatus

ev3 = EV3Brick()
ev3.screen.clear()
startangle = Gyro.angle()
while Button.CENTER not in ev3.buttons.pressed():
  pass
print(Gyro.angle() - startangle)

Gyro.reset_angle(0)
starttime = time.time()

baystatus = []

# start to pickup water to red box
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
gurn(67, aggresion=90, tp='circle', speed=200)
sweep(sensor=LeftColor, direction="right")
lfpidBlack(sensor=LeftColor, sideofsensor='in', blacks=1, speed=160, kp=0.4)
straight(30)
gurn(90, fb="forward", tp='pivot', speed=200)
lfpidBlack(sensor=LeftColor, sideofsensor='in', blacks=1, speed=100)
straight(-20)
gurn(-45, fb="forward", tp='pivot', speed=200)
gurn(45, fb="backward", tp='pivot', speed=200)
straight(-75)
robot.stop()

# red box
markingBlockColor = colorScan(acceptable=[Color.GREEN, Color.WHITE], direction='in')
straight(20)
gurn(-90, fb="backward", tp="pivot", speed=200)
straight(315)
RightMotor.run_angle(-150, 30)
color = colorScan(acceptable=[Color.BLACK, Color.RED, Color.YELLOW], direction='in')
if color != None:
  baystatus.append({"type" : "laundry", "color" : color})
RightMotor.run_angle(-150, -30)
if color == None:
  if markingBlockColor == Color.GREEN: # ball
    straight(-20)
    gurn(90, fb="backward", tp="pivot", speed=200)
    grab(oc="open")
    straight(-125)
    grab(oc="close")
    straight(20)
    lift(ud="up")
    gurn(80, fb="backward", tp="pivot", speed=200)
    straight(-120)
    lift(ud="downhalf")
    grab(oc="open")
    straight(70)
    lift(ud="downhalf")
    grab(oc="close")
    straight(120)
    grab(oc="open")
    straight(-70)
    grab(oc="close")
    straight(70)


  else: # water
    pass
else:
  straight(30)
  gurn(-38, fb="forward", tp="pivot", speed=200)
  grab(oc="open")
  straight(-150)
  if markingBlockColor == Color.GREEN: # ball
    gurn(42, fb="backward", tp="pivot", speed=200)
    straight(-75)
    grab(oc="close")
    straight(20)
    lift(ud="up")
    gurn(90, fb="backward", tp="pivot", speed=200)
    straight(-70)
    lift(ud="downhalf")
    grab(oc="open")
    straight(60)
    lift(ud="downhalf")
    grab(oc="close")
    straight(20)
    gurn(70, fb="forward", tp="pivot", speed=200)
    straight(160)
  elif markingBlockColor == Color.WHITE: # water
    grab(oc="close")
    straight(30)
    gurn(100, fb="backward", tp="pivot", speed=200)
    gurn(-45, fb="forward", tp="pivot", speed=200)
    straight(-60)
    baystatus = frombay(baystatus, {"type" : "water"}, "back")
    straight(-30)
    lift(ud="downhalf")
    grab(oc="open")
    straight(50)
    lift(ud="downhalf")
    grab(oc="close")
    straight(40)
    gurn(90, fb="forward", tp="pivot", speed=200)

# red box to green box
sweep(sensor=RightColor, direction="left")
lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, speed=160)


print(time.time() - starttime)