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

from threading import Thread

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
white = 41
threshold = int((black + white) / 2) #19


import time

def linesquare():
    lightrange = range(threshold - 2, threshold + 2)
    sped = -70
    rorl = "n"
    rightmotor.run(80)
    leftmotor.run(80)
    while rightcolor.reflection() > 4 and leftcolor.reflection() > 4:
        pass
    if rightcolor.reflection() < 4:
        rorl= "r" 
    if rorl == "r":
        while leftcolor.reflection() > 4:
            pass
        rightmotor.hold()
        leftmotor.hold()
        for i in range(3):
            if i % 2 == 0:
                rightmotor.run(sped)
                while rightcolor.reflection() not in lightrange:
                    pass
                rightmotor.hold()
            else:
                leftmotor.run(sped)
                while leftcolor.reflection() not in lightrange:
                    pass
                leftmotor.hold()
    else:
        while rightcolor.reflection() > 4:
            pass
        rightmotor.hold()
        leftmotor.hold()
        for i in range(3):
            if i % 2 == 0:
                leftmotor.run(sped)
                while leftcolor.reflection() not in lightrange:
                    pass
                leftmotor.hold()
            else:
                rightmotor.run(sped)
                while rightcolor.reflection() not in lightrange:
                    pass
                rightmotor.hold()


def line_follow(colour_sensor, speed, P_multiplier, D_multiplier, sample_time, distance):
    robot.reset()
    last_error = 0
    e_count = 0
    while robot.distance() < distance:
        p = 0
        d = 0
        error = 0

        error = threshold - colour_sensor.reflection()
        p = error * P_multiplier
        d = (error - last_error) / sample_time * D_multiplier
        #print('p:', p, 'd:', d, 'error:', error, 'ecount:', e_count, 'speed:', speed)
        #this stays on the right side of the line
        robot.drive(speed, (p + d))
        last_error = error
        time.sleep(sample_time)
    robot.stop()
    leftmotor.hold()
    rightmotor.hold()

#positive number turns counterclockwise
def gyro_turn(angle, sped=150):
    gyroSensor.reset_angle(0)
    speed = 0
    if angle < 0:
        while gyroSensor.angle() > angle:
            speed = sped*(1 - (gyroSensor.angle() / angle * 0.7)) if sped*(1 - (gyroSensor.angle() / angle * 0.7))>50 else 50
            robot.drive(0, speed)

    else:
        while gyroSensor.angle() < angle:
            speed = -sped*(1 - (gyroSensor.angle() / angle * 0.7)) if -sped*(1 - (gyroSensor.angle() / angle * 0.7))<-50 else -50
            robot.drive(0, speed)
    robot.stop()
    leftmotor.hold()
    rightmotor.hold()

#'f' for front, nothing for back
#def turntoangle(record, targetangle=0):
#    if record = True:
#        gyroSensor.reset_angle(0)
#    elif record = False:
#        gyroangle = gyroSensor.angle()
#        if targetangle < 0:
#            gyro_turn(targetangle - (0 - gyroangle))
#        else:
#            gyro_turn(targetangle + (0 - gyroangle))
#4 cm/360 degrees if not changing direction
#desmond != distance
def straight(desmond, sped):
    robot.reset()
    gyroSensor.reset_angle(0)
    while abs(robot.distance()) < desmond:
        robot.drive(sped, gyroSensor.angle())
    robot.stop()
#calibrate gyro (self = gyro to calibrate)
def calibrategs(self):
    print('GS Calibration begin')
    wait(500)
    for _ in range(2):
        gyro_speed = gyroSensor.speed()
        print(gyro_speed)
        gyro_angle_1 = gyroSensor.angle()
        print(gyro_angle_1)
        wait(500)
    gsnow = gyro_angle_1 = gyroSensor.angle()
    print('GS Calibration finish ' + str(gsnow))

calibrategs(gyroSensor)

linesquare()
