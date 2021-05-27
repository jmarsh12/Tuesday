import cv2 as cv

# file directories would be inserted into quotes
img = cv.imread('')
cv.imshow('')


def rescale_frame(frame, scale=0.75):
    """
    Rescales the frame for faster processing (images, video, and live video)
    :param frame: image for processing
    :param scale: scale of the image
    :return: returns the newly scaled image
    """
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

resized_image = rescaleFrame(img)
cv.imshow('', resized_image)


def change_res(width, height):
    # only works for live video (from camera)
    capture.set(3, width)
    capture.set(4, height)

# file directory goes inside the quotes
capture = cv.VideoCapture('')

"""
This while loop plays a video and rescales it according to the function rescaleFrame.
The loop runs for 20 seconds or if the user presses the 'd' button.
"""
while True:
    isTrue, frame = capture.read()

    frame_resized = rescale_frame(frame, scale = 0.2)

    cv.imshow('', frame)
    cv.imshow('', frame_resized)

    if cv.waitKey(20) & 0xFF == ord('d'):
        break

capture.release()
cv.destroyAllWindows()
