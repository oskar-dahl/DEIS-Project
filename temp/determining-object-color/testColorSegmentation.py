import cv2

image = cv2.imread("robot.jpg")

resized = cv2.resize(image, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_CUBIC)
image = resized
ratio = image.shape[0] / float(resized.shape[0])

blurred = cv2.GaussianBlur(image, (5, 5), 0)

## convert to hsv
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

#Blue
mask1 = cv2.inRange(hsv, (177/2, 40, 40), (197/2, 255, 255))

#Red
mask2 = cv2.inRange(hsv, (340/2, 40, 40), (360/2, 255, 255))

## final mask and masked
mask = cv2.bitwise_or(mask1, mask2)
target = cv2.bitwise_and(image, image, mask=mask)

cv2.imwrite("target.png", target)
