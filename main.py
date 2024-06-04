# TODO: DEFINE PRAGMAS

# Constants

DEFAULT_SPEED = 20 # % speed
COLOR_RANGE = 4 # Range for varying color
BLUE_RANGE = 5 # Range for varying blue color
GREEN_RANGE = 7 # Range for varying green color
DIRECT_COLOR_RANGE = 3 # Range for varying direct color
SCAN_SPEED = 8 # Speed for rotating robot to scan

# Surface thresholds
SURFACE_GREEN = 45
SURFACE_BLUE = 24

# Green thresholds
GREEN_GREEN = 42
GREEN_BLUE = 16
DIRECT_GREEN_GREEN = 38
DIRECT_GREEN_BLUE = 8

# Blue thresholds
BLUE_GREEN = 22
BLUE_BLUE = 19
DIRECT_BLUE_GREEN = 9
DIRECT_BLUE_BLUE = 11

def setBothMotorSpeed(speed):
	resetMotorEncoder(leftMotor)
	resetMotorEncoder(rightMotor)
	setMotorSpeed(leftMotor, speed)
	setMotorSpeed(rightMotor, speed)

def stopOnObstacle():
	setMotorSpeed(leftMotor, 0)
	setMotorSpeed(rightMotor, 0)
	sleep(1000)
	playTone(2000, 200)
	sleep(2000)

def setBothMotorTargets(left, right):
	resetMotorEncoder(leftMotor)
	resetMotorEncoder(rightMotor)

	setMotorTarget(leftMotor, left, DEFAULT_SPEED)
	setMotorTarget(rightMotor, right, DEFAULT_SPEED)

	waitUntilMotorStop(leftMotor)
	waitUntilMotorStop(rightMotor)

# Push obstacle away for ~10 cm
def pushObstacle():
	distanceToMove = 265

	# Move forward to touch obstacle (~10 cm distance)
	setBothMotorTargets(distanceToMove, distanceToMove)
	sleep(40)

	# Rotate robot to an angle
	setBothMotorTargets(180, -180)
	sleep(40)

	# Move obstacle away at an angle (~10 cm distance)
	setBothMotorTargets(distanceToMove, distanceToMove)

	# Reverse back to original position behind angle
	setBothMotorTargets(-distanceToMove - 5, -distanceToMove - 5)
	sleep(40)

	# Rotate robot back to original angle
	setBothMotorTargets(-163, 163)
	sleep(40)

	# Reverse back to original position
	setBothMotorTargets(-distanceToMove, -distanceToMove)

# Turn around for ~360 degrees
def turnAround():
	setBothMotorTargets(380, -380) # Value based on current robot implementation

# Scan left
def scanLeft(redValue, greenValue, blueValue):
	for i in range(0, 300, 10):
		setBothMotorTargets(-SCAN_SPEED, SCAN_SPEED)
		getColorRGB(colorSensor, redValue, greenValue, blueValue)
		sleep(50)

		if ((greenValue <= BLUE_GREEN + BLUE_RANGE & greenValue >= BLUE_GREEN - BLUE_RANGE
			& blueValue <= BLUE_BLUE + COLOR_RANGE & blueValue >= BLUE_BLUE - COLOR_RANGE)
			| (greenValue <= GREEN_GREEN + GREEN_RANGE & greenValue >= GREEN_GREEN - GREEN_RANGE
			& blueValue <= GREEN_BLUE + COLOR_RANGE & blueValue >= GREEN_BLUE - COLOR_RANGE)):
			# Exit loop and move
			break
		
	resetMotorEncoder(leftMotor)
	resetMotorEncoder(rightMotor)

# Scan right
def scanRight(redValue, greenValue, blueValue):
	for i in range(0, 300, 10):
		setBothMotorTargets(SCAN_SPEED, -SCAN_SPEED)
		getColorRGB(colorSensor, redValue, greenValue, blueValue)
		sleep(50)

		if ((greenValue <= BLUE_GREEN + BLUE_RANGE & greenValue >= BLUE_GREEN - BLUE_RANGE
			& blueValue <= BLUE_BLUE + COLOR_RANGE & blueValue >= BLUE_BLUE - COLOR_RANGE)
			| (greenValue <= GREEN_GREEN + GREEN_RANGE & greenValue >= GREEN_GREEN - GREEN_RANGE
			& blueValue <= GREEN_BLUE + COLOR_RANGE & blueValue >= GREEN_BLUE - COLOR_RANGE)):
			# Exit loop and move
			break
	
	resetMotorEncoder(leftMotor)
	resetMotorEncoder(rightMotor)

def performTask(greenValue, blueValue):
	# Blue line task
	if (greenValue <= BLUE_GREEN + COLOR_RANGE & greenValue >= BLUE_GREEN - COLOR_RANGE
		& blueValue <= BLUE_BLUE + COLOR_RANGE & blueValue >= BLUE_BLUE - COLOR_RANGE):
		stopOnObstacle()
		turnAround()
	# Green line task
	elif (greenValue <= GREEN_GREEN + COLOR_RANGE & greenValue >= GREEN_GREEN - COLOR_RANGE
			& blueValue <= GREEN_BLUE + COLOR_RANGE & blueValue >= GREEN_BLUE - COLOR_RANGE):
		stopOnObstacle()
		pushObstacle()
	
def findPath(redValue, greenValue, blueValue):
	# Direct colored line
	if ((greenValue <= DIRECT_GREEN_GREEN + DIRECT_COLOR_RANGE & greenValue >= DIRECT_GREEN_GREEN - DIRECT_COLOR_RANGE
		& blueValue <= DIRECT_GREEN_BLUE + DIRECT_COLOR_RANGE & blueValue >= DIRECT_GREEN_BLUE - DIRECT_COLOR_RANGE)
		| (greenValue <= DIRECT_BLUE_GREEN + DIRECT_COLOR_RANGE & greenValue >= DIRECT_BLUE_GREEN - DIRECT_COLOR_RANGE
		& blueValue <= DIRECT_BLUE_BLUE + DIRECT_COLOR_RANGE & blueValue >= DIRECT_BLUE_BLUE - DIRECT_COLOR_RANGE)):
		scanLeft(redValue, greenValue, blueValue)
	# Surface
	elif (greenValue <= SURFACE_GREEN + COLOR_RANGE & greenValue >= SURFACE_GREEN - COLOR_RANGE
			& blueValue <= SURFACE_BLUE + COLOR_RANGE & blueValue >= SURFACE_BLUE - COLOR_RANGE):	
		scanRight(redValue, greenValue, blueValue)

	# Indirect colored line
	else:
		setBothMotorSpeed(DEFAULT_SPEED)

def main():
	leftPos, rightPos
	distance
	redValue, greenValue, blueValue

	# Init
	resetMotorEncoder(leftMotor)
	resetMotorEncoder(rightMotor)

	# Run
	while (true):
		# Color Sensor
		getColorRGB(colorSensor, redValue, greenValue, blueValue)

		# Ultrasonic Sensor - Stop robot based on distance
		distance = getUSDistance(ultrasonicSensor)
		if (distance <= 10.0):
			performTask(greenValue, blueValue)
		else:
			findPath(redValue, greenValue, blueValue)

		# Display motor speeds
		leftPos = getMotorEncoder(leftMotor)
		rightPos = getMotorEncoder(rightMotor)

		# Text
		displayCenteredTextLine(1, "Red: %d", redValue)
		displayCenteredTextLine(2, "Green: %d", greenValue)
		displayCenteredTextLine(3, "Blue: %d", blueValue)
		displayCenteredTextLine(4, "Distance: %f", distance)
		displayCenteredTextLine(5, "Left motor: %d", leftPos)
		displayCenteredTextLine(6, "Right motor: %d", rightPos)
