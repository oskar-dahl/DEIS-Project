#!/usr/bin/env python
# USAGE
# python detect_color.py --image example_shapes.png

# NOT USED IN FINAL VERSION

# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
from pyimagesearch.colorlabeler import ColorLabeler
from tools import euclideanDistance
from collections import Iterable
# import argparse
import cv2
import math

def getGPSPos(image, ROBOT='blue'):

	blurred = cv2.GaussianBlur(image, (5, 5), 0)

	## convert to hsv
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	if ROBOT == 'blue':
		#Blue
		mask = cv2.inRange(hsv, (177/2, 100, 40), (197/2, 255, 255))

	if ROBOT == 'red':
		#Red
		mask = cv2.inRange(hsv, (340/2, 100, 40), (360/2, 255, 255))

	blurred = cv2.bitwise_and(image, image, mask=mask)

	gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
	lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
	thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]

	# find contours in the thresholded image
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if cnts is None:
		return 0, 0, 0

	cnts = cnts[1]                                                                                                         

	# initialize the shape detector and color labeler
	sd = ShapeDetector()
	cl = ColorLabeler()

	_, ySize = mask.shape[:2]

	if isinstance(cnts, Iterable):
		# loop over the contours
		for c in cnts:

			# compute the center of the contour
			M = cv2.moments(c)

			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
			else:
				cX = 0
				cY = 0

			# detect the shape of the contour and label the color
			shape = sd.detect(c)
			color = cl.label(lab, c)

			if shape == "triangle" and color == ROBOT:

				# multiply the contour (x, y)-coordinates by the resize ratio,
				# then draw the contours and the name of the shape and labeled
				# color on the image
				c = c.astype("float")
				c = c.astype("int")

				# Find sticky point
				maxDistance = 0
				for point in c:
					distance = euclideanDistance(point[0], [cX, cY])
					if distance > maxDistance:
						maxDistance = distance
						stickyPoint = point

				pixelAngle = math.atan2((ySize-cY) - (ySize-stickyPoint[0][1]), cX - stickyPoint[0][0])

				if pixelAngle < 0:
					pixelAngle = 2 * math.pi + pixelAngle

				return cX, cY, pixelAngle
			
			else:
				return 0, 0, 0
		else:
			return 0, 0, 0

			#text = "{} {}".format(color, shape)
			#cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
			#cv2.putText(image, text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

			#cv2.circle(image, tuple([cX, cY]), 3, (255, 0, 0), -1)
			#cv2.circle(image, tuple(stickyPoint[0]), 3, (0, 0, 255), -1)
			#cv2.line(image, tuple([cX, cY]), tuple(stickyPoint[0]), (255, 255, 255), 1)

			#print("Shape = " + shape + "| Color = " + color)
			#print("cX = " + str(cX) + " | cY = " + str(cY))
			#print("sX = " + str(stickyPoint[0][0]) + " | sY =" + str(stickyPoint[0][1]))
			#print("A = " + str(pixelAngle*180/math.pi))
			#print("---------------------")
