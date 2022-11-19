#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
import time
import _thread

GrabMotor = Motor(Port.A)
RightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
LeftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
LiftMotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# Gyro = GyroSensor(Port.S1)
LeftColor = ColorSensor(Port.S2)
RightColor = ColorSensor(Port.S3)
FrontColor = ColorSensor(Port.S4)

robot = DriveBase(LeftMotor, RightMotor, wheel_diameter=94.2, axle_track=157)
robot.settings(straight_speed=300)

def lfBlack(Sensor, ProportionalGain=1):
  Threshold = (8 + 76) / 2 # Black  = 8, White = 76
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    if(Sensor == LeftColor):
      Deviation = - (LeftColor.reflection() - Threshold)
    if(Sensor == RightColor):
      Deviation = (RightColor.reflection() - Threshold)
    turn_rate = ProportionalGain * Deviation
    robot.drive(180, turn_rate)

  robot.stop()

def lfDistance(sensor, distance, sideofsensor, proportionalGain=0.5, startncap=[]):
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')

  threshold = (8 + 76) / 2 # Black  = 8, White = 76
  sideofsensor = -1 if sideofsensor == 'in' else 1
  robot.reset()

  if startncap == []:
    speed = 160
  else:
    speed = list(range(startncap[0], startncap[1]))
    
  lastdistance = 0
  num = 0
  while abs(robot.distance()) < distance:
    if abs(lastdistance - robot.distance()) > distance / len(speed):
      lastdistance = robot.distance()
      num += 1
    if(sensor == LeftColor):
      Deviation = - (LeftColor.reflection() - threshold) * sideofsensor
    if(sensor == RightColor):
      Deviation = (RightColor.reflection() - threshold) * sideofsensor
    turn_rate = proportionalGain * Deviation
    robot.drive(speed[num], turn_rate)

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
    if sideofsensor == 'in':
      error = target - sensor.reflection()
    elif sideofsensor == 'out':
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

def lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, waitdistance=25, kp=0.25, ki=0, kd=0.5, startncap=[], estdistance=0, threshold='x',): # wait distance is the # of mm after a black it waits until continue detecting blacks
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
    if (LeftColor.reflection() + RightColor.reflection()) / 2 < threshold and lastdistance + waitdistance < abs(robot.distance()):
      count += 1
      lastdistance = abs(robot.distance())

    if abs(lastdistancechange - robot.distance()) > estdistance / len(speed) and num < len(speed) - 1:
      lastdistancechange = robot.distance()
      num += 1

    if sideofsensor == 'in':
      error = target - sensor.reflection()
    elif sideofsensor == 'out':
      error = sensor.reflection() - target
    integral += error
    derivative = error - lasterror
    turn = kp * error + ki * integral + kd * derivative
    lasterror = error

    robot.drive(speed[num], turn)

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
  return signa * signb * (dividend % divisor)

def orient(angle, fb, tp, speed=30):
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  if tp not in ['tank', 'pivot', 'circle']:
    raise Exception('tp must be "tank" or "pivot" or "circle"')
  gurn(fraudulo(angle, 360) - fraudulo(robot.angle(), 360), speed=speed, tp=tp, fb=fb)

def sTurn(rl, fb, turn, tp='pivot', drive=0, turnSpeed=100): # rl = right-left, fb = forward-backward, turn = turn degrees(posotive), drive = drive between turns(positive)
  if rl not in ['right', 'left']:
    raise Exception('rl must be "right" or "left"')
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')
  
  if rl == 'right':
    turn *= -1
  if fb == 'backward':
    drive *= -1
    conificient = 1
  else:
    conificient = -1

  startangle = robot.angle()

  gurn(turn, tp=tp, fb=fb, speed=turnSpeed)
  if drive != 0:
    straight(drive)
  gurn(conificient * (startangle - robot.angle()), tp=tp, fb=fb, speed=turnSpeed)

def straight(distance):
  robot.straight(distance)
  robot.stop()

def sweep(sensor, direction, speed=50):
  if sensor not in [RightColor, LeftColor]:
    raise Exception('sensor must be RightColor or LeftColor')
  if direction not in ['right', 'left']:
    raise Exception('direction must be "right" or "left"')

  startangle = robot.angle()
  info = []
  target = (8 + 76) / 2 # Black  = 8, White = 76
  
  if direction == 'right':
    robot.drive(0, speed)
  else:
    robot.drive(0, -speed)
  
  while sensor.reflection() > 15:
    info.append([sensor.reflection(), robot.angle()])

  robot.stop()

  if info == []:
    return None # maybe add opposite later

  reflection = []
  for i in info:
    reflection.append(i[0])
 
  maxindex = reflection.index(max(reflection))
  info, reflection = info[maxindex:], reflection[maxindex:]

  closest = float('inf')
  closestindex = 0
  for i in reflection:
    if abs(i - target) < closest:
      closest = abs(i - target)
      closestindex = reflection.index(i)

  targetangle = info[closestindex][1]

  gurn(robot.angle() - targetangle, tp='tank', fb='forward', speed=speed)

def grab(oc='open', percentage=1, pinch=True):
  if oc == 'open':
    GrabMotor.stop()
    time.sleep(0.1)
    GrabMotor.run_angle(400, 200 * percentage)
  elif oc == 'close':
    GrabMotor.run(-400)
    time.sleep(0.6 * percentage)
    if not pinch:
      GrabMotor.stop()

def grabasync(oc='open', percentage=1, pinch=True):
  _thread.start_new_thread(grab, (oc, percentage, pinch))

def lift(ud='up', percentage=1):
  if ud == 'up':
    LiftMotor.run_angle(400, -320 * percentage)
  elif ud == 'uphalf':
    LiftMotor.run_angle(400, -160)
  elif ud == 'upfull':
    LiftMotor.run_angle(400, -550)
  elif ud == 'down':
    LiftMotor.run_angle(400, 320 * percentage)
  elif ud == "downhalf":
    LiftMotor.run_angle(400, 160)
  elif ud == "downfull":
    LiftMotor.run_angle(400, 550)

def liftasync(ud='up', percentage=1):
  _thread.start_new_thread(lift, (ud, percentage))

def colorScan(acceptable, direction):
  if FrontColor.color() in acceptable:
    return FrontColor.color()
  else:
    startangle = robot.angle()

    if direction == 'in':
      RightMotor.run(-150)
    elif direction == 'out':
      RightMotor.run(150)

    color = None
    while color not in acceptable and abs(startangle - robot.angle()) < 40:
      color = FrontColor.color()

    robot.stop()

    if direction == 'in':
      RightMotor.run(75)
      while robot.angle() > startangle:
        pass
    elif direction == 'out':
      RightMotor.run(-75)
      while robot.angle() < startangle:
        pass

    robot.stop()
    return color

def newColorScan(acceptable, direction):
  if FrontColor.color() in acceptable:
    return FrontColor.color()
  else:
    startangle = robot.angle()

    if direction == 'in':
      RightMotor.run(-150)
    elif direction == 'out':
      RightMotor.run(150)

    color = None
    while color not in acceptable and abs(startangle - robot.angle()) < 40:
      color = FrontColor.color()

    robot.stop()
    return color

def LineSquaring(Num):
  THRESHOLD = (8 + 76) / 2
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

def frombay(baystatus, object, position, liftheight="full", grabstatus="close", liftstatus=0, pickup=True, realdistance=None, constant=None):
  if object["type"] not in ["water", "laundry"]:
    raise Exception('object must be {"type" : "water"} or {"type" : "laundry", "color" : (Color.RED or Color.YELLOW or Color.Black)}')
  if position not in ["front", "back"]:
    raise Exception('position must be "front" or "back"')
  if liftheight not in ["full", "half"] and type(liftheight) != float:
    raise Exception('lift must be "full" or "half" or a number')
  if grabstatus not in ["close", "open"]:
    raise Exception('grabstatus must be "close" or "open"')
  if pickup not in [True, False]:
    raise Exception('pickup must be True or False')
  if not pickup:
    if realdistance == None:
      raise Exception('realdistance must be a number')
    if realdistance < 0:
      raise Exception('realdistance must be positive')
  if constant not in [None, "blueball", "bluewater"]:
    raise Exception('constant must be None or "blueball" or "bluewater"')
  
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
  
  blueballconstant = -5
  bluewaterconstant = -15

  if constant == "blueball":
    dis += blueballconstant
  elif constant == "bluewater":
    dis += bluewaterconstant

  if grabstatus == "close":
    grab(oc='open')
  straight(dis)
  lift(ud="down", percentage=liftstatus)
  grab(oc='close')
  if pickup:
    if liftheight == "full":
      lift(ud='up')
    elif liftheight == "half":
      lift(ud='up', percentage=0.5)
    else:
      lift(ud='up', percentage=liftheight)
    straight(-dis - 35)

    baystatus.remove(object)

  else:
    print("finish distance: " + str(realdistance - dis))
    straight(realdistance - dis)

  return baystatus

def getfirstlaundrycolor(baystatus):
  for i in reversed(baystatus):
    if i["type"] == "laundry":
      return True, i["color"]
  return False, None

def slack(speed, fb):
  if fb not in ['forward', 'backward']:
    raise Exception('fb must be "forward" or "backward"')

  if fb == "backward":
    speed *= -1

  robot.drive(speed, 0)
  while (LeftColor.reflection() + RightColor.reflection()) / 2 > 15:
    pass
  robot.stop()

ev3 = EV3Brick()
ev3.screen.clear()
# startangle = Gyro.angle()
while Button.CENTER not in ev3.buttons.pressed():
  pass
# print(Gyro.angle() - startangle)

# Gyro.reset_angle(0)
starttime = time.time()

baystatus = []

# pick up yellow water
liftasync(ud="downfull")
straight(-270)
gurn(-45, fb="backward", tp="tank", speed=100)
slack(speed=300, fb="backward")
straight(-60)
grab(oc="close")
grabasync(oc="open")
straight(-150)
grab(oc="close")
baystatus.append({"type": "water"})

# blue bay green marking block
gurn(-90, fb="backward", tp="tank", speed=100)
grabasync(oc="open")
straight(-230)
grab(oc="close")
grabasync(oc="open")
straight(-100)
grab(oc="close")
baystatus.append({"type": "laundry", "color" : "red"})

# drive to green laundry dropoff
lfpidBlack(sensor=LeftColor, sideofsensor='in', blacks=1, startncap=100)
straight(20)
gurn(-90, fb="forward", tp="pivot", speed=200)
lfpidBlack(sensor=LeftColor, sideofsensor='out', blacks=1, startncap=100)
gurn(90, fb="forward", tp="pivot", speed=200)
lfpidDistance(distance=100, sensor=RightColor, sideofsensor='out')
gurn(180, fb="forward", tp="tank", speed=100)
straight(-100)
baystatus = frombay(baystatus, {"type" : "water"}, "back")
straight(-30)
lift(ud="down", percentage=0.6)
grab(oc="open")

# to red and green
straight(50)
grabasync(oc="close")
lift(ud="down", percentage=0.4)
sweep(LeftColor, "left")
lfpidBlack(sensor=LeftColor, sideofsensor='out', blacks=1, startncap=100)
straight(20)
gurn(90, fb="forward", tp="pivot", speed=200)
lfpidDistance(distance=300, sensor=LeftColor, sideofsensor='out')
straight(200)
sweep(RightColor, "left")
lfpidBlack(sensor=RightColor, sideofsensor='in', blacks=1, startncap=100, kp=0.4)
gurn(135, fb="forward",tp ="tank")
straight(-250)
gurn(55, fb="forward", tp="tank")
grabasync(oc="open")
straight(-150)

# green bay water
grab(oc="close")
grabasync(oc="open")
straight(-150)
grab(oc="close")
baystatus.append({"type": "water"})

# red bay green marking block
gurn(-90, fb="backward", tp="tank", speed=100)
grabasync(oc="open")
straight(-230)
grab(oc="close")
grabasync(oc="open")
straight(-150)
grab(oc="close")
baystatus.append({"type": "laundry", "color" : "red"})

# drive to red laundry dropoff
lfpidBlack(sensor=LeftColor, sideofsensor='in', blacks=1, startncap=100)
straight(20)
gurn(-90, fb="forward", tp="pivot", speed=200)
lfpidBlack(sensor=LeftColor, sideofsensor='out', blacks=1, startncap=100)
gurn(-90, fb="forward", tp="pivot", speed=200)
lfpidDistance(distance=100, sensor=RightColor, sideofsensor='out')
gurn(210, fb="forward", tp="tank", speed=100)
straight(-100)
baystatus = frombay(baystatus, {"type" : "water"}, "back")
straight(-20)
lift(ud="down", percentage=0.6)
grab(oc="open")

# back to base
straight(70)
grabasync(oc="close")
lift(ud="down", percentage=0.4)
sweep(LeftColor, "right")
lfpidBlack(sensor=LeftColor, sideofsensor='in', blacks=1, startncap=100)
straight(20)
gurn(-90, fb="forward", tp="pivot", speed=200)
lfpidDistance(distance=300, sensor=LeftColor, sideofsensor='out')
gurn(45, fb="forward", tp="tank", speed=100)

print(time.time() - starttime)