#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Variables and constants
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
leftcolor = ColorSensor(Port.S2)
gyroSensor = GyroSensor(Port.S3)
black = 3
white = 35
threshold = int((black + white) / 2) #19
print(threshold)
#seeing white first and then line squaring
def linesquare():
    lightrange = range(threshold - 2, threshold + 2) #light range for final result
    #lightrange = range(threshold - 4, threshold + 4) #light rnage for adjustment
    
    #First, move the robot straight until one of the sensor sees range. 
    while rightcolor.reflection() > 4 and leftcolor.reflection() > 4:
        robot.drive(70, 0)
    robot.stop()
    #counting variable
    r_white = 0
    l_white = 0

    #Second, adjust left and right motor in turns until both sensors are in light range
    while rightcolor.reflection() not in lightrange or leftcolor.reflection() not in lightrange:
        while rightcolor.reflection() not in lightrange: #use adjustment range
            #Let robot see white first, use counting variable to make sure it only run once
            while rightcolor.reflection() < white - 2 and r_white < 1:
                rightmotor.run(60)
                r_white += 1
            
            #manuver based on reading compared to threshold value
            if rightcolor.reflection() > threshold:
                rightmotor.run(60)
                print('right F: ', rightcolor.reflection())
            elif rightcolor.reflection() < threshold:
                rightmotor.run(-60)
                print('right B: ', rightcolor.reflection())

        #motor brake
        rightmotor.brake()    
        
        #repeat the above for the sensor and motor on left side
        while leftcolor.reflection() not in lightrange: 

            while leftcolor.reflection() < white - 2 and l_white < 1:
                leftmotor.run(60)
                l_white += 1
            if leftcolor.reflection() > threshold:
                leftmotor.run(60)
                print("left F: ", leftcolor.reflection())
            elif leftcolor.reflection() < threshold:
                leftmotor.run(-60)
                print("left B: ", leftcolor.reflection())

        leftmotor.brake()
linesquare()
   



