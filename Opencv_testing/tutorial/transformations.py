import cv2 as cv
import numpy as np

img = cv.imread('images/Cartoon Forest.jpg')

cv.imshow('Forest', img)


# translation
def translate(img, x, y):
    trans_matrix = np.float32([[1, 0, x], [0, 1, y]])
    dimensions = (img.shape[1], img.shape[0])
    return cv.warpAffine(img, trans_matrix, dimensions)


# -x --> left
# -y --> up
# x --> right
# y --> down

translated = translate(img, -100, 100)
cv.imshow('Translated', translated)


# rotation
def rotate(img, angle, rotation_point=None):
    (height, width) = img.shape[:2]
    if rotation_point is None:
        rotation_point = (width // 2, height // 2)

    rotation_matrix = cv.getRotationMatrix2D(rotation_point, angle, 1.0)
    dimensions = (width, height)
    return cv.warpAffine(img, rotation_matrix, dimensions)


rotated = rotate(img, -45)
cv.imshow('Rotated', rotated)

# don't do this!!!
rotated_rotated = rotate(rotated, -45)
cv.imshow('Rotated_rotated', rotated_rotated)


# resizing
resized = cv.resize(img, (500, 500), interpolation=cv.INTER_AREA)
cv.imshow('Resized', resized)


# flipping
# 0 = vertical flip, 1 = horizontal flip, -1 = both vertical and horizontal flip
flip = cv.flip(img, 0)
cv.imshow('Flip', flip)


# cropping
cropped = img[200:400, 300:400]
cv.imshow('Cropped', cropped)

cv.waitKey(0)
