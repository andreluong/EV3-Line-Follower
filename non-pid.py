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
DEFAULT_SPEED = 60 # speed (mm/s)
COLOR_RANGE = 4 # Range for varying color
BLUE_RANGE = 5 # Range for varying blue color
GREEN_RANGE = 5 # Range for varying green color
DIRECT_COLOR_RANGE = 3 # Range for varying direct color
SCAN_SPEED = 8 # Speed for rotating robot to scan

# Surface thresholds
SURFACE_GREEN = 42
SURFACE_BLUE = 57

# Green thresholds
GREEN_GREEN = 37
GREEN_BLUE = 37
DIRECT_GREEN_GREEN = 34 # 32
DIRECT_GREEN_BLUE = 15 # 28

# Blue thresholds
BLUE_GREEN = 30 
BLUE_BLUE = 48
DIRECT_BLUE_GREEN = 8 # 13	# 18
DIRECT_BLUE_BLUE = 25 # 30  # 36

def scanLeft():
	for i in range(0, 300, 10):
		robot.turn(-SCAN_SPEED)
		redValue, greenValue, blueValue = colorSensor.rgb()
		wait(50)

		if ((greenValue <= 33 and blueValue <= 48) and (greenValue >= 20 and blueValue >= 38)) or ((greenValue <= 39 and blueValue <= 40) and (greenValue >= 35 and blueValue >= 29)):
			break # Exit loop and move


# Scan right
def scanRight():
	for i in range(0, 300, 10):
		robot.turn(SCAN_SPEED)
		redValue, greenValue, blueValue = colorSensor.rgb()
		wait(50)

		if ((greenValue <= 33 and blueValue <= 48) and (greenValue >= 20 and blueValue >= 38)) or ((greenValue <= 39 and blueValue <= 40) and (greenValue >= 35 and blueValue >= 29)):
			break # Exit loop and move


def pathFinder(greenValue, blueValue):
    # surface
	color = colorSensor.color()
	if color == Color.BLUE or color == Color.GREEN:
	# if greenValue >= 40 and blueValue >= 52:
		scanRight()
		ev3.speaker.beep(4000, 100)
	elif (greenValue <= 33 and blueValue <= 48) or (greenValue <= 39 and blueValue <= 40):
		robot.drive(40, 0)
	elif (greenValue <= 20 and blueValue <= 38) or (greenValue <= 35 and blueValue <= 29):
		scanLeft()
  
  
  
  

def stopOnObstacle():
	robot.straight(0)
	wait(1000)
	ev3.speaker.beep(2000, 200)
	wait(2000)

# Push obstacle away for ~10 cm
def pushObstacle():
	TEN_CENTIMETERS = 110
	TURN_ANGLE = 45

	# Move forward to touch obstacle (~10 cm distance)
	robot.straight(TEN_CENTIMETERS)

	# Rotate robot to an angle
	robot.turn(TURN_ANGLE)

	# Move obstacle away at an angle (~10 cm distance)
	robot.straight(TEN_CENTIMETERS)

	# Reverse back to original position behind angle
	robot.straight(-TEN_CENTIMETERS - 5)

	# Rotate robot back to original angle
	robot.turn(-TURN_ANGLE)

# Turn around for ~360 degrees
def turnAround():
	robot.turn(400) # Value based on current robot implementation

# Checks if robot is on blue line
def isOnBlueLine(greenValue, blueValue):
	return (greenValue <= BLUE_GREEN + COLOR_RANGE and greenValue >= BLUE_GREEN - COLOR_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE)

# Checks if robot is on green line
def isOnGreenLine(greenValue, blueValue):
	return (greenValue <= GREEN_GREEN + GREEN_RANGE and greenValue >= GREEN_GREEN - GREEN_RANGE
		and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE)

# Scan left
# def scanLeft():
# 	for i in range(0, 300, 10):
# 		robot.turn(-SCAN_SPEED)
# 		redValue, greenValue, blueValue = colorSensor.rgb()
# 		wait(50)

# 		if (isOnGreenLine(greenValue, blueValue) or isOnBlueLine(greenValue, blueValue)):
# 			robot.drive(DEFAULT_SPEED, 0)
# 			break # Exit loop and move

# # Scan right
# def scanRight():
# 	for i in range(0, 300, 10):
# 		robot.turn(SCAN_SPEED)
# 		redValue, greenValue, blueValue = colorSensor.rgb()
# 		wait(50)

# 		if (isOnGreenLine(greenValue, blueValue) or isOnBlueLine(greenValue, blueValue)):
# 			robot.drive(DEFAULT_SPEED, 0)
# 			break # Exit loop and move

def performTask(greenValue, blueValue):
	# Blue line task
	if (greenValue <= BLUE_GREEN + COLOR_RANGE and greenValue >= BLUE_GREEN - COLOR_RANGE
		and blueValue <= BLUE_BLUE + COLOR_RANGE and blueValue >= BLUE_BLUE - COLOR_RANGE):
		stopOnObstacle()
		turnAround()
	# Green line task
	elif (greenValue <= GREEN_GREEN + GREEN_RANGE and greenValue >= GREEN_GREEN - GREEN_RANGE
			and blueValue <= GREEN_BLUE + COLOR_RANGE and blueValue >= GREEN_BLUE - COLOR_RANGE):
		stopOnObstacle()
		pushObstacle()
	
def findPath(redValue, greenValue, blueValue):
	# Direct colored line
	if ((greenValue <= DIRECT_GREEN_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_GREEN_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_GREEN_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_GREEN_BLUE - DIRECT_COLOR_RANGE)
		or (greenValue <= DIRECT_BLUE_GREEN + DIRECT_COLOR_RANGE and greenValue >= DIRECT_BLUE_GREEN - DIRECT_COLOR_RANGE
		and blueValue <= DIRECT_BLUE_BLUE + DIRECT_COLOR_RANGE and blueValue >= DIRECT_BLUE_BLUE - DIRECT_COLOR_RANGE)):
		scanLeft()
	# Surface
	elif (greenValue <= SURFACE_GREEN + COLOR_RANGE and greenValue >= SURFACE_GREEN - COLOR_RANGE
			and blueValue <= SURFACE_BLUE + COLOR_RANGE and blueValue >= SURFACE_BLUE - COLOR_RANGE):	
		scanRight()
	# Indirect colored line
	else:
		robot.drive(DEFAULT_SPEED, 0)

# Main program
while True:
	# Color Sensor
	redValue, greenValue, blueValue = colorSensor.rgb()

	# Check colours
	print("Red: ", redValue)
	print("Green: ", greenValue)
	print("Blue: ", blueValue)
	print("")
 
	# pathFinder(greenValue, blueValue)
 
	robot.drive(40, 50)

	# robot.drive(20, 0)
	# Ultrasonic Sensor - Stop robot based on distance
	# if (ultrasonicSensor.distance() <= 100.0):
	# 	performTask(greenValue, blueValue)
	# else:
	# 	findPath(redValue, greenValue, blueValue)

	# robot.drive(10, 0)
	# findPath(redValue, greenValue, blueValue)
