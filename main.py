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
#Version notes: 3
#removed turntoangle

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
accangle = 0 # accumulated angle, angle you're supposed to be at. 
log = False

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

def linesquare(speed=150):
    lightrange = range(threshold - 2, threshold + 2)
    sped = -speed
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
    global log
    robot.reset()
    last_error = 0
    iteration = 0
    p = 0
    d = 0
    i = 0
    error = 0
    total = (p + (i*I_multiplier) + d)
    if log == True:
        data = DataLog('total', 'p', 'i', 'd', 'error', 'last error')
    while robot.distance() < distance:
        error = threshold - colour_sensor.reflection()
        p = error * P_multiplier
        d = (error - last_error) / sample_time * D_multiplier
        i = error * sample_time + i
        total = (p + (i*I_multiplier) + d)
        if log == True:
            data.log(total, p, i*I_multiplier, d, error, last_error)
        #this stays on the right side of the line
        robot.drive(speed, total)
        last_error = error
        time.sleep(sample_time)
    robot.stop()
    leftmotor.hold()
    rightmotor.hold()

#positive number turns counterclockwise
def turntoangle(Target_angle, gyroturn=False, sped=150, turntimes=1):
    #if gyroturn is false, target angle is the exact angle to turn to.
    #if gyroturn is true, target angle is how much you want to turn.
    global accangle
    print('target angle before', Target_angle, 'accangle', accangle, 'gyro angle', gyroSensor.angle())
    if gyroturn == True:
        accangle += Target_angle
    else:
        accangle = Target_angle
    for i in range(turntimes):
        if Target_angle != 0:
            print('target angle after', Target_angle, 'accangle', accangle, 'gyro angle', gyroSensor.angle())
            turnspeed = 0
            if Target_angle < gyroSensor.angle():
                while gyroSensor.angle() > Target_angle + 1:
                    turnspeed = sped*(1 - (gyroSensor.angle() / Target_angle * 0.7)) if sped*(1 - (gyroSensor.angle() / Target_angle * 0.7))>50 else 30
                    #if the proportional speed is > 50, then use the proportional speed. if not, then use 30.
                    robot.drive(0, turnspeed)

            else:
                while gyroSensor.angle() < Target_angle - 1:
                    turnspeed = -sped*(1 - (gyroSensor.angle() / Target_angle * 0.7)) if -sped*(1 - (gyroSensor.angle() / Target_angle * 0.7))<-50 else -30
                    #if the proportional speed is < 50, then use the proportional speed. if not, then use -30.
                    robot.drive(0, turnspeed)
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

#calibrategs(gyroSensor)

while True:
    ev3.screen.clear()
    #share model, basketvball
    if Button.UP in ev3.buttons.pressed():
        
        #ev3.speaker.say('Run 1')
        #Run 1
        accangle = 0
        gyroSensor.reset_angle(0)
        straight(200, 200) # moving to follow line that leads to line that leads to share model
        line_follow(rightcolor, 200, 1.8, 0.08, 0.1, 0.01, 200) #  follow line that leads to line that leads to share model
        straight(100, 200) # drive forwards a bit
        turntoangle(60, True) # turn to follow line that leads to share model
        line_follow(rightcolor, 200, 1.5, 0.08, 0.1, 0.01, 330) 
        straight(50, 130, sampleFactor=3) # drops red cube
        straight(30, -200)
        turntoangle(31.5, True)
        mediummotor.run_angle(800, 1190) #arm goes down
        oldstraight(90, 150)#move into basketball
        wait(500)
        straight(20, -100)
        #turntoangle(-1, True)
        mediummotor.run_angle(-700, 21.9 * 90)#arm goes up
        straight(30, -200)
        turntoangle(39, True)
        straight(700, 200, sampleFactor=3)

    # slide
    #elif Button.RIGHT in ev3.buttons.pressed():
    #    while Button.RIGHT in ev3.buttons.pressed():
    #        pass
    #    #ev3.speaker.say('Run 2 waiting')
    #    while not Button.RIGHT in ev3.buttons.pressed():


    elif Button.RIGHT in ev3.buttons.pressed():

        #ev3.speaker.say('Run 2a')
        #bench
        accangle = 0
        gyroSensor.reset_angle(0)
        oldstraight(400, 250)
        oldstraight(100, 150)
        wait(500)
        oldstraight(500, -500)

        while Button.CENTER not in ev3.buttons.pressed():
            pass

        #ev3.speaker.say('Run 2b')
        #slide
        accangle = 0
        gyroSensor.reset_angle(0)
        straight(550, 350, Kd=0.04)
        wait(1000)
        while robot.distance() > -100:
            robot.drive(-200, -2)
        while robot.distance() > -500:
            robot.drive(-250, 20)
        robot.stop()

        while Button.CENTER not in ev3.buttons.pressed():
            pass
        #treadmill
        accangle = 0
        gyroSensor.reset_angle(0)
        straight(270, 200)                                                                                                                         
        line_follow(rightcolor, 175, 1.6, 0.04, 0.1, 0.01, 1200)# arives at treadmill
        #turntoangle(0.01)
        robot.drive(95, -1.5) #80, -2
        mediummotor2.run_time(300, 1000, wait=True)#rolls up platform
        wait(900) #1500
        robot.stop()
        leftmotor.hold()
        rightmotor.hold()
        mediummotor2.run_time(1000, 3500)#runs treadmill
        accangle = 1
        oldstraight(2000, -500)# goes back
        #health units, minifigure on tire, weight machine, drop bricks
    elif Button.LEFT in ev3.buttons.pressed():
        
        # set point 1 for gyro (parallel to south wall)
        accangle = 0
        gyroSensor.reset_angle(0)

        #health units
        straight(250, 200) #move to first line follow
        line_follow(rightcolor, 200, 1.6, 0.02, 0.1, 0.01, 420) #linefollow to health unit spot
        turntoangle(21)#adjust to health unit
        mediummotor.run_angle(500, 6.5 * 90)# drop health units
        straight(50, -100)# moving to get health unit out of claw
        mediummotor.run_angle(500, 5.5 * 90) # getting health unit out of claw
        straight(50, -100) # move back to line

        #Minifigure
        mediummotor.run_angle(400, -5 * 90, wait=False) # moving arm up for minifig
        wait(300)
        line_follow(rightcolor, 100, 1.5, 0.02, 0.1, 0.01, 150) # making sure robot stays on line
        log = True
        line_follow(rightcolor, 200, 1.5, 0.02, 0.1, 0.01, 820) # linefollow to treadmill
        log = False
        # turn to angle
        turntoangle(90) #turning so bot is perpendicular to line
        #gyro_turn(90)
        straight(190, 300, 3) #moving to big tire
        #mediummotor.run_angle(400, -5 * 90)
        turntoangle(139) #turning to drop minifigure
        wait(500)
        mediummotor.run_angle(250, 9 * 90) # dropping minifigure

        #weight machine
        mediummotor.run_angle(500, -10 * 90, wait=False)#preparing for weight machine
        turntoangle(90, sped=100)#turns to linesquare
        turntoangle(90, sped=100)#again, just to make sure
        wait(700)
        straight(120, 200)#moving close to line
        linesquare()#linesquaring
        linesquare()#again, just to make sure
        wait(500)
        turntoangle(78) # turns from linesquare to line up with weight machine
        wait(500)
        oldstraight(140, 150) #move to weight machine
        while Button.CENTER not in ev3.buttons.pressed():
            pass
        robot.turn(5) # turning so arm touches weight machine
        while Button.CENTER not in ev3.buttons.pressed():
            pass
        mediummotor.run_angle(500, 17 * 90) #weight machine push down
        mediummotor.run_angle(-500, 15 * 90, wait=False)# weight machine release

        #drop boccia bricks
        turntoangle(180, sped=100) # turn to follow line beside weight machine
        log = True
        #line_follow(leftcolor, 200, 1.3, 0.03, 1, 0.01, 100)
        line_follow(leftcolor, 200, 1.3, 0.03, 1, 0.01, 320) #follow line beside weight machine 
        log = False
        straight(580, 200, 3) # go a bit further, the line squigles
        turntoangle(225) #turn so back is facing target
        straight(290, -300) # backing in to release bricks
        mediummotor2.run_angle(1000, -355)#releasing bricks
        wait(500)

        #back to home
        mediummotor2.run_angle(800, 370, wait=False) # bringing box bottom back up, for testing purposes
        straight(1200, 500) # back to home


        #step counter, through pull-up bar, dance
    elif Button.DOWN in ev3.buttons.pressed():
        
        #ev3.speaker.say('Run 4')
        #step counter, pullup bar, dance
        accangle = 0
        gyroSensor.reset_angle(0)
        wait(400)
        robot.reset()
        while robot.distance() < 910:
            robot.drive(250, 5)
        robot.stop()
        straight(1170-800, 500) #cross brigde
        straight(50, -200)
        turntoangle(90)
        straight(150, -250)
        straight(500, 400)
        turntoangle(35, True)
        straight(500, 250)
        straight(30, -200)
        while True: #dance
            straight(50, -200)
            straight(50, 200)

    #elif Button.CENTER in ev3.buttons.pressed():
    #    accangle = 0
    #    gyroSensor.reset_angle(0)
    #    straight(270, 200)                                                                                                                         
    #    line_follow(rightcolor, 200, 1.6, 0.04, 0.1, 0.01, 1200)
    #    #turntoangle(0.01)
    #    robot.drive(80, -1)
    #    mediummotor2.run_time(300, 1000)
    #    wait(800)
    #    robot.stop()
    #    leftmotor.hold()
    #    rightmotor.hold()
    #    mediummotor2.run_time(1000, 2500)
    #    accangle = 0
    #    straight(2000, -500)
    #ev3.screen.print('none')