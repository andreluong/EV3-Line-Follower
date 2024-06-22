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
servoMotor = Motor(Port.B)
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.5, axle_track=104.5)
robot.settings(250, 250, 500, 500)

# Constants
DEFAULT_SPEED = 180 # mm/s
COLOR_RANGE = 9 # Range for varying color
DIRECT_COLOR_RANGE = 5 # Range for varying direct color

# Surface thresholds
SURFACE_RED = 37
SURFACE_GREEN = 42
SURFACE_BLUE = 57

# Green thresholds
GREEN_RED = 21
GREEN_GREEN = 38
GREEN_BLUE = 36
DIRECT_GREEN_RED = 8
DIRECT_GREEN_GREEN = 36 # 32
DIRECT_GREEN_BLUE = 16 # 28

# Blue thresholds
BLUE_RED = 16
BLUE_GREEN = 18
BLUE_BLUE = 36
DIRECT_BLUE_RED = 4
DIRECT_BLUE_GREEN = 7 # 13	# 18
DIRECT_BLUE_BLUE = 25 # 30  # 36

# PID
Kp = 6.75 # Proportional gain 
Ki = 0.005  # Integral gain
Kd = 13.5 # Derivative gain
MOTOR_SPEED_LIMIT = 250 # mm/s

# Variables
previousError = 0
sumOfErrors = 0

# Checks if robot is on blue line
def isOnBlueLine(greenValue, blueValue):
	return (greenValue <= BLUE_GREEN + COLOR_RANGE and greenValue >= BLUE_GREEN - COLOR_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE)

# Checks if robot is on green line
def isOnGreenLine(greenValue, blueValue):
	return (greenValue <= GREEN_GREEN + COLOR_RANGE and greenValue >= GREEN_GREEN - COLOR_RANGE
		and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE)

def isOnDirectBlueLine(greenValue, blueValue):
    return (greenValue <= DIRECT_GREEN_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_GREEN_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_GREEN_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_GREEN_BLUE - DIRECT_COLOR_RANGE)

def isOnDirectGreenLine(greenValue, blueValue):
    return  (greenValue <= DIRECT_BLUE_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_BLUE_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_BLUE_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_BLUE_BLUE - DIRECT_COLOR_RANGE)

def isOnSurface(greenValue, blueValue):
    return (greenValue <= SURFACE_GREEN + COLOR_RANGE and greenValue >= SURFACE_GREEN - COLOR_RANGE
        and blueValue <= SURFACE_BLUE + COLOR_RANGE and blueValue >= SURFACE_BLUE - COLOR_RANGE)


# Calculates the error
def calculateError(redValue, greenValue, blueValue):    
    # Helps the robot turn right appropriately
    RIGHT_TURN_CONSTANT = -42
    
    if isOnDirectGreenLine(greenValue, blueValue):
        error = RIGHT_TURN_CONSTANT - redValue - DIRECT_GREEN_RED / 2
    elif isOnDirectBlueLine(greenValue, blueValue):
        error = RIGHT_TURN_CONSTANT - redValue - DIRECT_BLUE_RED / 2
    elif (isOnGreenLine(greenValue, blueValue)):
        error = redValue - GREEN_RED
    elif (isOnBlueLine(greenValue, blueValue)):
        error = redValue - BLUE_RED
    else:
        error = redValue - SURFACE_RED / 2
	
    return error

# Calculates the PID control output
def calculatePID(error, previousError):
    integral = Ki * sumOfErrors  # Update integral term with sum of errors
    derivative = Kd * (error - previousError)  # Calculate derivative term
    pidValue = Kp * error + integral + derivative
    return pidValue, integral

def stopOnObstacle():
    leftMotor.run(0)
    rightMotor.run(0)
    ev3.speaker.beep(2000, 2000)
    wait(2000)

# Push obstacle
def pushObstacle():
    robot.straight(100)
    robot.turn(-10)
    servoMotor.run_time(-1500, 710, then=Stop.HOLD, wait=True)
    robot.stop()

# Turn around for ~180 degrees
def turnAround():
    robot.turn(180)
    robot.stop()
    
def performTask(greenValue, blueValue):
    stopOnObstacle()
    if isOnGreenLine(greenValue, blueValue) or isOnDirectBlueLine(greenValue, blueValue):
        pushObstacle()
    else:
        turnAround()

def findPath(redValue, greenValue, blueValue):
    error = calculateError(redValue, greenValue, blueValue)

    pidValue, sumOfErrors = calculatePID(error, previousError)
    previousError = error

    # Set motor speeds with adjustment
    leftSpeed = DEFAULT_SPEED + pidValue
    rightSpeed = DEFAULT_SPEED - pidValue

    # Ensure speeds are within valid range
    leftSpeed = max(min(leftSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)
    rightSpeed = max(min(rightSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)

    leftMotor.run(leftSpeed)
    rightMotor.run(rightSpeed)


# Run the robot
while True:
    redValue, greenValue, blueValue = colorSensor.rgb()
        
    # Check for obstacles within 10 cm
    if (ultrasonicSensor.distance() <= 100.0):
        performTask(greenValue, blueValue)
    else:
        findPath(redValue, greenValue, blueValue)        