# accelerating line following
def lfDistance(sensor, distance, sideofsensor, proportionalGain=0.5, startncap=[]):
  if sideofsensor not in ['in', 'out']:
    raise Exception('sideofsensor must be "in" or "out"')

  threshold = (9 + 74) / 2 # Black = 9, White = 74
  sideofsensor = -1 if sideofsensor == 'in' else 1
  robot.reset()

  if startncap == []:
    speed = 160
  else:
    speed = list(range(startncap[0], startncap[1]))
    '''
    speed = []
    speed.append(startncap[0])
    while startncap[1] != speed[-1]:
      if startncap[0] - startncap[1] < 0:
        speed.append(speed[-1] + 1)
      else:
        speed.append(speed[-1] - 1)
    '''
      
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