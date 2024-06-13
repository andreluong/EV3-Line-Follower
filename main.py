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
DEFAULT_SPEED = 30 # speed (mm/s)
COLOR_RANGE = 4 # Range for varying color
BLUE_RANGE = 5 # Range for varying blue color
GREEN_RANGE = 7 # Range for varying green color
DIRECT_COLOR_RANGE = 3 # Range for varying direct color
SCAN_SPEED = 8 # Speed for rotating robot to scan

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



def stopOnObstacle():
	robot.straight(0)
	wait(1000)
	ev3.speaker.beep(2000, 200)
	wait(2000)

# TODO:
def setBothMotorTargets(left, right):
	robot.drive(DEFAULT_SPEED, left)

	# setMotorTarget(leftMotor, left, DEFAULT_SPEED)
	# setMotorTarget(rightMotor, right, DEFAULT_SPEED)

# Push obstacle away for ~10 cm
def pushObstacle():
	distanceToMove = 265

	# Move forward to touch obstacle (~10 cm distance)
	robot.straight(100)

	# Rotate robot to an angle
	robot.turn(180)
	# setBothMotorTargets(180, -180)

	# Move obstacle away at an angle (~10 cm distance)
	robot.straight(100)

	# Reverse back to original position behind angle
	robot.straight(-100)

	# Rotate robot back to original angle
	# setBothMotorTargets(-163, 163)
	robot.turn(-163)

# Turn around for ~360 degrees
def turnAround():
	# setBothMotorTargets(380, -380) # Value based on current robot implementation
	robot.turn(380) # Value based on current robot implementation

# Checks if robot is on blue line
def isOnBlueLine(greenValue, blueValue):
	return (greenValue <= BLUE_GREEN + BLUE_RANGE and greenValue >= BLUE_GREEN - BLUE_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE)

# Checks if robot is on green line
def isOnGreenLine(greenValue, blueValue):
	return (greenValue <= GREEN_GREEN + GREEN_RANGE and greenValue >= GREEN_GREEN - GREEN_RANGE
		and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE)

# Scan left
def scanLeft(redValue, greenValue, blueValue):
	for i in range(0, 300, 10):
		robot.turn(-SCAN_SPEED)
		redValue, greenValue, blueValue = colorSensor.rgb()
		wait(30)

		if (isOnGreenLine(greenValue, blueValue) or isOnBlueLine(greenValue, blueValue)):
			break # Exit loop and move

# Scan right
def scanRight(redValue, greenValue, blueValue):
	for i in range(0, 300, 10):
		robot.turn(SCAN_SPEED)
		redValue, greenValue, blueValue = colorSensor.rgb()
		wait(30)

		if (isOnGreenLine(greenValue, blueValue) or isOnBlueLine(greenValue, blueValue)):
			break # Exit loop and move

def performTask(greenValue, blueValue):
	# Blue line task
	if (greenValue <= BLUE_GREEN + COLOR_RANGE and greenValue >= BLUE_GREEN - COLOR_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE):
		stopOnObstacle()
		turnAround()
	# Green line task
	elif (greenValue <= GREEN_GREEN + COLOR_RANGE and greenValue >= GREEN_GREEN - COLOR_RANGE
			and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE):
		stopOnObstacle()
		pushObstacle()
	
def findPath(redValue, greenValue, blueValue):
	# Direct colored line
	if ((greenValue <= DIRECT_GREEN_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_GREEN_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_GREEN_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_GREEN_BLUE - DIRECT_COLOR_RANGE)
		or (greenValue <= DIRECT_BLUE_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_BLUE_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_BLUE_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_BLUE_BLUE - DIRECT_COLOR_RANGE)):
		scanLeft(redValue, greenValue, blueValue)
	# Surface
	elif (greenValue <= SURFACE_GREEN + COLOR_RANGE and greenValue >= SURFACE_GREEN - COLOR_RANGE
			and blueValue <= SURFACE_BLUE + COLOR_RANGE and blueValue >= SURFACE_BLUE - COLOR_RANGE):	
		scanRight(redValue, greenValue, blueValue)
	# Indirect colored line
	else:
		robot.drive(DEFAULT_SPEED, 0)

# Main program
while True:
	# Color Sensor
	redValue, greenValue, blueValue = colorSensor.rgb()

	# Check colours
	# print("Red: ", redValue)
	# print("Green: ", greenValue)
	# print("Blue: ", blueValue)
	# print("")

	# Ultrasonic Sensor - Stop robot based on distance
	# if (ultrasonicSensor.distance() <= 10.0):
	# 	performTask(greenValue, blueValue)
	# else:
	# 	findPath(redValue, greenValue, blueValue)

	findPath(redValue, greenValue, blueValue)
