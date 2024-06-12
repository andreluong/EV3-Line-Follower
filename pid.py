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
leftMotor = Motor(Port.A)
rightMotor = Motor(Port.D)
colorSensor = ColorSensor(Port.1)
ultrasonicSensor = UltrasonicSensor(Port.2)

# Initialize drive base
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.5, axle_track=104)

# Constants
DEFAULT_SPEED = 20  # % speed

# PID
Kp = 2  # Proportional gain constant (adjust for responsiveness)
Kd = 0.1  # Derivative gain constant (adjust for stability)
Ki = 0.01  # Integral gain constant (adjust for smoothness)
MOTOR_SPEED_LIMIT = 100

# Variables
previousError = 0
sumOfErrors = 0



# Calculates the error (deviation from center of green line)
def calculateError():
    redValue, greenValue, blueValue = colorSensor.rgb()

	if (isOnGreenLine(green_value)):
		error = (GREEN_ON_GREEN - greenValue) / GREEN_RANGE
	elif (isOnBlueLine(blue_value)):
		error = (BLUE_ON_BLUE - blueValue) / BLUE_RANGE
	else:
		error = 0
	
	return error

# Calculates the PID control output
def calculatePID(error, previousError):
    integral = Ki * sumOfErrors  # Update integral term with sum of errors
    derivative = Kd * (error - previousError)  # Calculate derivative term
    pidValue = Kp * error + integral + derivative
    return pidValue, integral


while (True):
    error = calculateError()
    
    pidValue, sumOfErrors = calculatePID(error, previousError)
    previousError = error

    # Set motor speeds with adjustment
    leftSpeed = DEFAULT_SPEED + pidValue
    rightSpeed = DEFAULT_SPEED - pidValue

    # Ensure speeds are within valid range
    leftSpeed = max(min(leftSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)
    rightSpeed = max(min(rightSpeed, MOTOR_SPEED_LIMIT), -MOTOR_SPEED_LIMIT)
    
    robot.drive(DEFAULT_SPEED, leftSpeed, rightSpeed)