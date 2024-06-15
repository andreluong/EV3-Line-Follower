#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
								InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Initialize ojects
ev3 = EV3Brick()
leftMotor = Motor(Port.D)
rightMotor = Motor(Port.A)
colorSensor = ColorSensor(Port.S4)
ultrasonicSensor = UltrasonicSensor(Port.S1)

# Initialize drive base
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.5, axle_track=104)


while True:
    
    color = colorSensor.color()
    
    if (color == Color.BLUE):
        robot.stop()
        ev3.speaker.beep(5000, 100)
        wait(2000)
    
    
    if (color == Color.GREEN):
        robot.stop()
        ev3.speaker.beep(2000, 200)
        wait(2000)
    