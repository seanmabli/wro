#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
import time
import _thread

# motor1 = Motor(Port.A)
# motor2 = Motor(Port.B)
# motor3 = Motor(Port.C)
# motor4 = Motor(Port.D)

color1 = ColorSensor(Port.S1)
color2 = ColorSensor(Port.S2)
color3 = ColorSensor(Port.S3)
color4 = ColorSensor(Port.S4)

ev3 = EV3Brick()
ev3.screen.clear()

output = ""
starttime = time.time()

for z in range(0, 100):
  for i, colornum in enumerate([color1, color2, color3, color4]):
    output += str(i) + ": " + str(colornum.rgb())
  output += "\n"

print("Time: " + str((time.time() - starttime)))
print(output)