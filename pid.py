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

# Constants
DEFAULT_SPEED = 20 # % speed
COLOR_RANGE = 4 # Range for varying color
BLUE_RANGE = 5 # Range for varying blue color
GREEN_RANGE = 7 # Range for varying green color
DIRECT_COLOR_RANGE = 3 # Range for varying direct color
SCAN_SPEED = 8 # Speed for rotating robot to scan

# PID
Kp = 2  # Proportional gain constant (adjust for responsiveness)
Kd = 0.1  # Derivative gain constant (adjust for stability)
Ki = 0.01  # Integral gain constant (adjust for smoothness)
MOTOR_SPEED_LIMIT = 100

# Variables
previousError = 0
sumOfErrors = 0



# NOTE: OTHER LAB ROOM VALUES

# Surface thresholds
SURFACE_GREEN = 47
SURFACE_BLUE = 42

# Green thresholds
GREEN_GREEN = 46
GREEN_BLUE = 28
DIRECT_GREEN_GREEN = 47
DIRECT_GREEN_BLUE = 15

# Blue thresholds
BLUE_GREEN = 27
BLUE_BLUE = 33
DIRECT_BLUE_GREEN = 16
DIRECT_BLUE_BLUE = 29



# Checks if robot is on blue line
def isOnBlueLine(greenValue, blueValue):
	return (greenValue <= BLUE_GREEN + COLOR_RANGE and greenValue >= BLUE_GREEN - COLOR_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE)

# Checks if robot is on green line
def isOnGreenLine(greenValue, blueValue):
	return (greenValue <= GREEN_GREEN + COLOR_RANGE and greenValue >= GREEN_GREEN - COLOR_RANGE
		and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE)





# Calculates the error (deviation from center of green line)
def calculateError():
    redValue, greenValue, blueValue = colorSensor.rgb()
    
    # Check colours
    print("Red: ", redValue)
    print("Green: ", greenValue)
    print("Blue: ", blueValue)
    print("")

    if (isOnGreenLine(greenValue, blueValue)):
        error = (GREEN_GREEN - greenValue) / COLOR_RANGE
        print("hit green")
    elif (isOnBlueLine(greenValue, blueValue)):
        error = (BLUE_BLUE - blueValue) / COLOR_RANGE
        print("hit blue")
    else:
        error = 0
	
    return error

# Calculates the PID control output
def calculatePID(error, previousError):
    integral = Ki * sumOfErrors  # Update integral term with sum of errors
    derivative = Kd * (error - previousError)  # Calculate derivative term
    pidValue = Kp * error + integral + derivative
    return pidValue, integral

while True:
    error = calculateError()

    print("Error: ", error)
    
    pidValue, sumOfErrors = calculatePID(error, previousError)
    previousError = error

    print("PidValue: ", pidValue)

    # Set motor speeds with adjustment
    leftSpeed = DEFAULT_SPEED + pidValue
    rightSpeed = DEFAULT_SPEED - pidValue

    # Ensure speeds are within valid range
    leftSpeed = max(min(leftSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)
    rightSpeed = max(min(rightSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)

    robot.drive(10, 0)
    wait(3000)
 
    print("")
    
    # robot.drive(DEFAULT_SPEED, leftSpeed, rightSpeed)
