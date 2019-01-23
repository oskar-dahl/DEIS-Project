# USAGE
# python detect_color.py --image example_shapes.png

# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
from pyimagesearch.colorlabeler import ColorLabeler
import argparse
import imutils
import cv2
import math

ROBOT="blue"

def euclideanDistance(vector1, vector2):
    '''calculate the euclidean distance, no numpy
    input: numpy.arrays or lists
    return: euclidean distance
    '''
    dist = [(a - b)**2 for a, b in zip(vector1, vector2)]
    dist = math.sqrt(sum(dist))
    return dist

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="path to the input image")
args = vars(ap.parse_args())

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread(args["image"])
resized = image
ratio = 1
#resized = imutils.resize(image, width=300)
#ratio = image.shape[0] / float(resized.shape[0])

blurred = cv2.GaussianBlur(resized, (5, 5), 0)

## convert to hsv
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#Blue
mask1 = cv2.inRange(hsv, (177/2, 100, 40), (197/2, 255, 255))

#Red
mask2 = cv2.inRange(hsv, (340/2, 100, 40), (360/2, 255, 255))

## final mask and masked
mask = cv2.bitwise_or(mask1, mask2)
blurred = cv2.bitwise_and(image, image, mask=mask)

# blur the resized image slightly, then convert it to both
# grayscale and the L*a*b* color spaces

gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]
#cv2.imshow("Thresh", thresh)
#cv2.waitKey(0)

# find contours in the thresholded image
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]

# initialize the shape detector and color labeler
sd = ShapeDetector()
cl = ColorLabeler()

_, ySize = blurred.shape[:2]

# loop over the contours
for c in cnts:

	# compute the center of the contour
	M = cv2.moments(c)

	if M["m00"] != 0:
		cX = int((M["m10"] / M["m00"]) * ratio)
		cY = int((M["m01"] / M["m00"]) * ratio)
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
		c *= ratio
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

		text = "{} {}".format(color, shape)
		cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
		cv2.putText(image, text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

		cv2.circle(image, tuple([cX, cY]), 3, (255, 0, 0), -1)
		cv2.circle(image, tuple(stickyPoint[0]), 3, (0, 0, 255), -1)
		cv2.line(image, tuple([cX, cY]), tuple(stickyPoint[0]), (255, 255, 255), 1)

		print("Shape = " + shape + "| Color = " + color)
		print("cX = " + str(cX) + " | cY = " + str(cY))
		print("sX = " + str(stickyPoint[0][0]) + " | sY =" + str(stickyPoint[0][1]))
		print("A = " + str(pixelAngle*180/math.pi))
		print("---------------------")

	#else:
		#print("No triangle was found")


cv2.imshow("Image", image)
cv2.waitKey(0)
