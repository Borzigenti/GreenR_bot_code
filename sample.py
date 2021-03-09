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
# Variables and cons

import multiprocessing

ev3 = EV3Brick()
leftmotor = Motor(Port.C, Direction.CLOCKWISE)
rightmotor = Motor(Port.B, Direction.CLOCKWISE)
#left medium motor
mediummotor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
#right medium motor
mediummotor2 = Motor(Port.A, Direction.COUNTERCLOCKWISE)
robot = DriveBase(leftmotor, rightmotor, 55.85, 150)
print(robot.settings())
robot.settings(200, 520, 200, 100)
rightcolor = ColorSensor(Port.S1)
leftcolor = ColorSensor(Port.S2)
gyroSensor = GyroSensor(Port.S3)
accangle = 0

black = 3
white = 41
threshold = int((black + white) / 2) #19

import time

def oldstraight(desmond, sped):
    robot.reset()
    #gyroSensor.reset_angle(0)
    while abs(robot.distance()) < desmond:
        robot.drive(sped, gyroSensor.angle() - accangle)
    robot.stop()

def linesquare():
    lightrange = range(threshold - 2, threshold + 2)
    sped = -150
    rorl = "n"
    rightmotor.run(150)
    leftmotor.run(150)
    while rightcolor.reflection() > 4 and leftcolor.reflection() > 4:
        pass
    if rightcolor.reflection() < 4:
        rorl= "r" 
    if rorl == "r":
        while leftcolor.reflection() > 4:
            pass
        rightmotor.hold()
        leftmotor.hold()
        for i in range(2):
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
        for i in range(2):
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

def line_follow(colour_sensor, speed, P_multiplier, D_multiplier, I_multiplier, sample_time, distance):
    robot.reset()
    last_error = 0
    e_count = 0
    while robot.distance() < distance:
        p = 0
        d = 0
        i = 0
        error = 0
        error = threshold - colour_sensor.reflection()
        p = error * P_multiplier
        d = (error - last_error) / sample_time * D_multiplier
        i = error + i
        #print('p:', p, 'd:', d, 'error:', error, 'ecount:', e_count, 'speed:', speed)
        #this stays on the right side of the line
        robot.drive(speed, (p + d + (i*I_multiplier)))
        last_error = error
        time.sleep(sample_time)
    robot.stop()
    leftmotor.hold()
    rightmotor.hold()
    #robot.reset()
    #last_error = 0
    #e_count = 0
    #while robot.distance() < distance:
    #    p = 0
    #    d = 0
    #    error = 0

    #    error = threshold - colour_sensor.reflection()
    #    p = error * P_multiplier
    #    d = (error - last_error) / sample_time * D_multiplier
    #    #print('p:', p, 'd:', d, 'error:', error, 'ecount:', e_count, 'speed:', speed)
    #    #this stays on the right side of the line
    #    robot.drive(speed, (p + d))
    #    last_error = error
    #    time.sleep(sample_time)
    #robot.stop()
    #leftmotor.hold()
    #rightmotor.hold()


#positive number turns counterclockwise
def turntoangle(Target_angle, gyroturn=False, sped=150, turntimes=1):
    global accangle
    print('target angle before', Target_angle, 'accangle', accangle, 'gyro angle', gyroSensor.angle())
    if gyroturn == True:
        Target_angle += accangle
        accangle = Target_angle
    else:
        accangle = Target_angle
    for i in range(turntimes):
        if Target_angle != 0:
            print('target angle mid', Target_angle, 'accangle', accangle, 'gyro angle', gyroSensor.angle())
            speed = 0
            if Target_angle < gyroSensor.angle():
                while gyroSensor.angle() > Target_angle + 1:
                    speed = sped*(1 - (gyroSensor.angle() / Target_angle * 0.7)) if sped*(1 - (gyroSensor.angle() / Target_angle * 0.7))>50 else 30
                    robot.drive(0, speed)

            else:
                while gyroSensor.angle() < Target_angle - 1:
                    speed = -sped*(1 - (gyroSensor.angle() / Target_angle * 0.7)) if -sped*(1 - (gyroSensor.angle() / Target_angle * 0.7))<-50 else -30
                    robot.drive(0, speed)
        print('target angle after', Target_angle, 'accangle', accangle, 'gyro angle', gyroSensor.angle())
        robot.stop()
        leftmotor.hold()
        rightmotor.hold()

#'f' for front, nothing for back

#4 cm/360 degrees if not changing direction
#desmond != distance
#def straight(desmond, sped):
#    print('straight begin')
#    robot.reset()
#    gyroSensor.reset_angle(0)
#    while abs(robot.distance()) < desmond:
#        if gyroSensor.angle() != 0:
#            robot.drive(sped, gyroSensor.angle() * 2)
#        else:
#            robot.drive(sped, 0)
#        print('gyroangle', gyroSensor.angle())
#    robot.stop()
#    print('straight finish')
def straight(desmond, sped, sampleFactor=4, Kd=0):
    global accangle
    error = 0
    last_error = 0
    turnrate = 0
    d = 0
    robot.reset()
    print('straight start')
    while abs(robot.distance()) < desmond:
        error = accangle - gyroSensor.angle()
        d = (error - last_error) / 0.01 * Kd
        turnrate = error * sampleFactor + d
        robot.drive(sped, -turnrate)
        #print('straight: gyro', gyroSensor.angle(), 'error', error, 'turnrate', -turnrate)
        last_error = error
    robot.stop()
    leftmotor.hold()
    rightmotor.hold()
    wait(10)
    print('straight end')

#calibrate gyro (self = gyro to calibrate)
def calibrategs(self):
    print('GS Calibration begin')
    wait(500)
    for _ in range(3):
        gyro_speed = gyroSensor.speed()
        print(gyro_speed)
        gyro_angle_1 = gyroSensor.angle()
        print(gyro_angle_1)
        wait(500)
    gsnow = gyro_angle_1 = gyroSensor.angle()
    print('GS Calibration finish ' + str(gsnow))
    global accangle
    error = 0
    turnrate = 0
    robot.reset()
    while abs(robot.distance()) < desmond:
        error = accangle - gyroSensor.angle()
        turnrate = error / 0.25
        robot.drive(sped, -turnrate)
    robot.stop()
    wait(10)


accangle = 0
gyroSensor.reset_angle(0)
straight(270, 200)                                                                                                                         
line_follow(rightcolor, 175, 1.6, 0.04, 0.1, 0.01, 1200)
#turntoangle(0.01)
robot.drive(80, -2)
mediummotor2.run_time(300, 1000)
wait(1500)
robot.stop()
leftmotor.hold()
rightmotor.hold()
mediummotor2.run_time(1000, 2500)
turntoangle(-1, True)
accangle = 1
oldstraight(2000, -500)


