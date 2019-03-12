#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys
import numpy as np
import cv2

from cscore import CameraServer, VideoSource, CvSource, VideoMode, CvSink, UsbCamera
from networktables import NetworkTablesInstance

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

#Setup color brightnesses
rBrightness = 160
gBrightness = 160
bBrightness = 160
rBrightnessMax = 255
gBrightnessMax = 255
bBrightnessMax = 255
frame = "global"
topX = "global"
topY = "global"
topX = -2
topY = -2
botX = "global"
botY = "global"
botX = -2
botY = -2
width = 320
height = 240
direction = "global"
direction = 0
alreadyFound = "global"
alreadyFound = False

"""Report parse error."""
def parseError(str):
	print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
	cam = CameraConfig()

	# name
	try:
		cam.name = config["name"]
	except KeyError:
		parseError("could not read camera name")
		return False

	# path
	try:
		cam.path = config["path"]
	except KeyError:
		parseError("camera '{}': could not read path".format(cam.name))
		return False

	# stream properties
	cam.streamConfig = config.get("stream")

	cam.config = config

	cameraConfigs.append(cam)
	return True

"""Read configuration file."""
def readConfig():
	global team
	global server

	# parse file
	try:
		with open(configFile, "rt") as f:
			j = json.load(f)
	except OSError as err:
		print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
		return False

	# top level must be an object
	if not isinstance(j, dict):
		parseError("must be JSON object")
		return False

	# team number
	try:
		team = j["team"]
	except KeyError:
		parseError("could not read team number")
		return False

	# ntmode (optional)
	if "ntmode" in j:
		str = j["ntmode"]
		if str.lower() == "client":
			server = False
		elif str.lower() == "server":
			server = True
		else:
			parseError("could not understand ntmode value '{}'".format(str))

	# cameras
	try:
		cameras = j["cameras"]
	except KeyError:
		parseError("could not read cameras")
		return False
	for camera in cameras:
		if not readCameraConfig(camera):
			return False
			
	return True

"""Start running the camera."""
def startCamera(config):
	print("Starting camera '{}' on {}".format(config.name, config.path))
	inst = CameraServer.getInstance()
	camera = UsbCamera(config.name, config.path)
	server = inst.startAutomaticCapture(camera=camera, return_server=True)

	camera.setConfigJson(json.dumps(config.config))
	camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

	if config.streamConfig is not None:
		server.setConfigJson(json.dumps(config.streamConfig))

	return camera

# read configuration
if not readConfig():
	sys.exit(1)
	
# start NetworkTables
ntinst = NetworkTablesInstance.getDefault()
print("Setting up NetworkTables client for team {}".format(team))
ntinst.startClientTeam(team)

# Smart Dashboard
SDV = ntinst.getTable('SmartDashBoard')

#def printPixel(x, y):
#	if (frame[y,x,0] > bBrightness and frame[y,x,1] > gBrightness and frame[y,x,2] > rBrightness):
#		SDV.putNumber(("Pixel " + str(y) + ", " + str(x) + " Reflective Tape:"), True)
#	else:
#		SDV.putNumber(("Pixel " + str(y) + ", " + str(x) + " Reflective Tape:"), False)

def testPixel(y, x):
	if (frame[x,y,0] > bBrightness and frame[x,y,0] < bBrightnessMax and frame[x,y,1] > gBrightness and frame[x,y,1] < gBrightnessMax and frame[x,y,2] > rBrightness and frame[x,y,2] < rBrightnessMax):
		return True;
	else:
		return False;
		
def findReflectiveTape():
	global alreadyFound
	for i in range(0, round(width / 10)):
		for j in range(0, round(height / 10)):
			if (testPixel((i * 10) + 5,(j * 10) + 5)):
				if (alreadyFound == False):
					findOuterBounds(((i * 10) + 5), ((j * 10) + 5), 1)

def findOuterBounds(x, y, a):
	#Top slope > -1 && < 1
	#Right & Left slope < -1 || > 1
	#If right & left are negative tape is on the right side.
	#Find slope to right top and left lines then come up with point
	#of intersection and then find exact top left and top right corners.
	#Use the top slope to find out wether it is left or right tape then check
	#to the right or left to find the other tape
	
	#top Line will set topX and topY
	leftX = -1
	rightX = -1
	topCornerFound = False
	bottomCornerFound = False
	bottomPixelFound = False
	perfectTopCornerFound = False
	firstLeft = False
	pixelFound = False
	rightPixel = False
	leftPixel = False
	counter = 10
	failSafe = 0
	failSafeTest = True
	tempLX = 0
	tempRX = 0
	previous = 0
	move = 8
	foundTopLine = True
	global topX
	global topY
	global botX
	global botY
	global direction
	global alreadyFound
	alreadyFound = True
	while (topCornerFound == False and foundTopLine and failSafeTest):
		foundTopLine = findTopLine(x, y)
		if (foundTopLine):
			leftX = findLeftSide(topX, topY)
			rightX = findRightSide(topX, topY)
			if (y == topY):
				topCornerFound = True
			y = topY
			x = round((rightX - leftX) / 2) + leftX
			failSafe = failSafe + 1
			if (failSafe == 100):
				failSafeTest = False
	failSafe = 0
	failSafeTest = True
	if (topCornerFound):
		while (perfectTopCornerFound == False and failSafeTest and rightX - leftX > 0):
			pixelFound = False
			for i in range(leftX, rightX):
				if (testPixel(i, topY - 1)):
					pixelFound = True
					if (firstLeft == False):
						firstLeft = True
						tempLX = i
					tempRX = i
			if (tempRX != 0 and tempLX != 0):
				rightX = tempRX
				leftX = tempLX
			if (pixelFound == False):
				perfectTopCornerFound = True
			else:
				topY = topY - 1
				if (topY < 0):
					topY = 0
					perfectTopCornerFound = True
			failSafe = failSafe + 1
			if (failSafe == 100):
				print("Failsafe triggered in topX topY perfect detection!")
				failSafeTest = False
		topX = round((rightX - leftX) / 2) + leftX #This line is dividing x by 2
		failSafe = 0
		failSafeTest = True
		while (direction == 0 and failSafeTest):
			if (topX - 6 >= 0 and topX + 6 <= width - 1 and topY + counter <= height - 1 and counter >= 0):
				leftPixel = testPixel(topX - 6, topY + counter)
				rightPixel = testPixel(topX + 6, topY + counter)
				if (leftPixel and rightPixel):
					counter = counter - 1
					if (previous == 1):
						direction = 3
					else:
						previous = -1
				elif (leftPixel and rightPixel == False):
					direction = -1
				elif (leftPixel == False and rightPixel):
					direction = 1
				else:
					counter = counter + 1
					if (previous == -1):
						direction = 3
					else:
						previous = 1
			else:
				direction = 2
			failSafe = failSafe + 1
			if (failSafe == 100):
				print("Failsafe triggered in direction identifier!")
				failSafeTest = False
		failSafe = 0
		failSafeTest = True
		botX = topX
		botY = topY
		if (direction == -1 or direction == 1):
			while (bottomCornerFound == False and failSafeTest):
				if (botX + direction < width - 1 and botX + direction > 0 and botY + 2 < height - 1 and botY > 0):
					if (testPixel(botX + direction, botY)):
						botX = botX + direction
					elif (testPixel(botX + direction, botY + 1)):
						botX = botX + direction
						botY = botY + 1
					elif (testPixel(botX + direction, botY + 2)):
						botX = botX + direction
						botY = botY + 1
					else:
						bottomCornerFound = True
				else:
					botX = -1
					botY = -1
					bottomCornerFound = True
				failSafe = failSafe + 1
				if (failSafe == 100):
					print("Failsafe triggered in direction identifier!")
					failSafeTest = False
#				bottomPixelFound = False
#				previous = 0
#				botX = botX + (move * direction)
#				botY = botY + move
#				while (bottomPixelFound == False):
#					if (botX > 0 and botX < width - 1 and botY > 0 and botY < height - 1):
#						if(testPixel(botX, botY)):
#							previous = 1
#							botY = botY - 1
#						else:
#							if (previous == 0):
#								bottomPixelFound = True
#								if (move == 1):
#									bottomPixelFound = True
#									bottomCornerFound = True
#								else:
#									move = round(move / 2)
#							else:
#								botY = botY + 1
#								bottomPixelFound = True
#					else:
#						botX = -1
#						botY = -1
#						bottomCornerFound = True
#						bottomPixelFound = True

#				for i in range(0, round(move / 2)):
#					if (botX > 0 and botX < width - 1 and botY > 0 and botY < height - 1):
#						if (testPixel(botX, botY)):
#							if (previous == 1):
#								bottomPixelFound = True
#							else:
#								previous = -1
#								botY = botY - 1
#							if (botY < 0):
#								botY = botY + 1
#						else:
#							if (previous == -1):
#								bottomPixelFound = True
#							else:
#								previous = 1
#							botY = botY + 1
#							if (botY > width):
#								botY = botY - 1
#						if (bottomPixelFound):
#							i = move
#						else:
#							if (move > 2):
#								move = round(move / 2)
#							else:
#								bottomCornerFound = True
#					else:
#						botX = -1
#						botY = -1
#						bottomCornerFound = True
		#print ("Top X: " + str(topX) + ", Top Y: " + str(topY))
		#print ("Direction: " + str(direction))
		#print ("Bot X: " + str(botX) + ", Bot Y: " + str(botY))

def findTopLine(x, y):
	global topX
	global topY
	if (testPixel(x, y - 10)):
		y = y - 10
	while ((y - 1) >= 0):
		if (testPixel(x, y - 1)):
			y = y - 1
		else:
			topX = x
			topY = y
			return True
	topX = -1
	topY = -1
	return False
	
def findLeftSide(x, y):
	leftFound = False
	tenMove = True
	fiveMove = True
	while (leftFound == False):
		if (tenMove == True and x - 10 >= 0 and testPixel(x - 10, y)):
			x = x - 10
		elif (fiveMove == True and x - 5 >= 0 and testPixel(x - 5, y)):
			x = x - 5
			tenMove = False
		elif (x > 0 and testPixel(x - 1, y)):
			x = x - 1
			tenMove = False
			fiveMove = False
		else:
			leftFound = True
	return x

def findRightSide(x, y):
	rightFound = False
	tenMove = True
	fiveMove = True
	while (rightFound == False):
		if (tenMove == True and x + 10 <= width - 1 and testPixel(x + 10, y)):
			x = x + 10
		elif (fiveMove == True and x + 5 <= width - 1 and testPixel(x + 5, y)):
			x = x + 5
			tenMove = False
		elif (x < width - 1 and testPixel(x + 1, y)):
			x = x + 1
			tenMove = False
			fiveMove = False
		else:
			rightFound = True
	return x
	
				
				
				
if __name__ == "__main__":
	if len(sys.argv) >= 2:
		configFile = sys.argv[1]

	# start cameras
	print("Connecting to camera")
	cs = CameraServer.getInstance()
	cs.enableLogging()
	Camera = UsbCamera('rPi Camera 0', 0)
	Camera.setResolution(width, height)
	#cs.addCamera(Camera)
	cs.startAutomaticCapture(camera=Camera, return_server=True)
	
	CvSink = cs.getVideo()
	outputStream = CvSource('Processed Frames', VideoMode.PixelFormat.kBGR, width, height, 28)
	img = np.zeros(shape=(width, height, 3), dtype=np.uint8)
	count = 0
	global topX
	global topY
	global botX
	global botY
	global direction
	global alreadyFound
	while True:
		topX = -2
		topY = -2
		botX = -2
		botY = -2
		direction = 0
		alreadyFound = False
		GotFrame, frame = CvSink.grabFrame(img)
		if GotFrame  == 0:
			outputStream.notifyError(CvSink.getError())
			continue
	
		SDV.putNumber(("Count"), count)
		#findReflectiveTape()
		print("Blue: " + str(frame[239,0,0]) + ", Green: " + str(frame[239,0,1]) + ", Red: " + str(frame[239,0,2]))
		count = count + 1
		SDV.putNumber(("TopX"), topX)
		SDV.putNumber(("TopY"), topY)
		SDV.putNumber(("BotX"), botX)
		SDV.putNumber(("BotY"), botY)
		SDV.putNumber(("Direction"), direction)