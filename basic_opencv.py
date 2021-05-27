import cv2 as cv

img = cv.imread('images/Cartoon Forest.jpg')

cv.imshow('Forest', img)

# converting to grayscale
# parameters:
#   image, color
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('Gray', gray)

# blur an image
# parameters:
#   image, blur scale (must be odd), border
blur = cv.GaussianBlur(img, (7, 7), cv.BORDER_DEFAULT)
cv.imshow('Blur', blur)

# edge cascade (find the edges in an image)
# parameters required:
#   image, threshold 1 and threshold 2
canny = cv.Canny(img, 125, 175)
cv.imshow('Canny', canny)

# dilate an image using structuring
# parameters required:
#   image, kernel size, iteration number
dilated = cv.dilate(canny, (7, 7), iterations=3)
cv.imshow('Dilated', dilated)

# eroding
# parameters are the same as dilated
eroded = cv.erode(dilated, (3, 3), iterations=1)
cv.imshow('Eroded', eroded)

# resize and crop
# parameters required:
#   image, resolution, interpolation
resize = cv.resize(img, (500, 500), interpolation=cv.INTER_CUBIC)
cv.imshow('Resized', resize)

# parameters required:
#   image and destinations
cropping = img[50:200, 200:400]
cv.imshow('Cropped', cropping)

cv.waitKey(0)