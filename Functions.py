def MotorHold():
  robot.stop()
  left_motor.hold()
  right_motor.hold()

def LineFollowing(Distance, Sensor):
  # Black = 9, White = 70
  # Circumfrence = 34.56
  threshold = (9 + 70) / 2

  PROPORTIONAL_GAIN = 4
  left_motor.reset_angle(0)
  right_motor.reset_angle(0)
  while (left_motor.angle() + right_motor.angle()) / 2 < (Distance / 174.36) * 360:
    if(Sensor == 'Left'):
      deviation = (left_color.reflection() - threshold)
    if(Sensor == 'Right'):
      deviation = (right_color.reflection() - threshold)
    turn_rate = PROPORTIONAL_GAIN * deviation
    robot.drive(-100, turn_rate)

  MotorHold()

def LineSquaring():
  THRESHOLD = (9 + 70) / 2
  # Move Forward To White
  while left_color.color() != Color.WHITE or right_color.color() != Color.WHITE:
    robot.drive(-50, 0)

  MotorHold()

  # If Left Color Detected The White
  if(left_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while left_color.reflection() >= THRESHOLD:
      robot.drive(-10, 0)
    MotorHold()
      
  # If Right Color Detected The White
  if(right_color.color() == Color.WHITE):  
    # Move Forward To THRESHOLD
    while right_color.reflection() >= THRESHOLD:
      robot.drive(-10, 0)
    MotorHold()

  # Move Left Or Right To Get Color Reflection Equal
  while right_color.reflection() > left_color.reflection():
    right_motor.run(-10)
    left_motor.run(10)
  MotorHold()
  while right_color.reflection() < left_color.reflection():
    right_motor.run(10)
    left_motor.run(-10)
  MotorHold()
  print(right_color.reflection())
  print(left_color.reflection())