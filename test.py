#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Variables and constants
length = 20
armpos = 1

ev3 = EV3Brick()
leftmotor = Motor(Port.C, Direction.CLOCKWISE)
rightmotor = Motor(Port.B, Direction.CLOCKWISE)
#left medium motor
mediummotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
#right medium motor
mediummotor2 = Motor(Port.A, Direction.COUNTERCLOCKWISE)
robot = DriveBase(leftmotor, rightmotor, 55.85, 108)
robot.settings(300, 100, 200, 100)
rightcolor = ColorSensor(Port.S1)
gyroSensor = GyroSensor(Port.S3)

black = 3
white = 41
threshold = int((black + white) / 2) #19

#34 degrees is one unit
#mediummotor.run_angle(200, 34 * length)
#robot.straight(50)
#mediummotor.run_angle(200, -34 * length)
while True:
    if Button.UP in ev3.buttons.pressed():
        mediummotor.run(600)
    elif Button.DOWN in ev3.buttons.pressed():
        mediummotor.run(-600)
    else:
        mediummotor.stop()
    if Button.LEFT in ev3.buttons.pressed():
        mediummotor2.run(1000)
    elif Button.RIGHT in ev3.buttons.pressed():
        mediummotor2.run(-1000)
    else:
        mediummotor2.stop()
