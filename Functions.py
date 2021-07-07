
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

