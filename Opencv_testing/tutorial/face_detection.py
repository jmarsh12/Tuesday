import cv2 as cv


def rescale_frame(frame, scale=0.5):
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


img_device = cv.VideoCapture(0)

while(True):

    ret, img = img_device.read()
    # img = cv.imread('images/man.webp')
    # cv.imshow('People', img)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # cv.imshow('Gray', gray)

    haar_cascade = cv.CascadeClassifier('haar_face.xml')

    # uses scale factor to detect the faces and return the rectangular coordinates of the faces
    #   minNeighbors determines how sensitive OpenCV is to noise
    faces_rectangle = haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=8)
    print(f'Number of faces found = {len(faces_rectangle)}')

    face_locations = ''

    for (x, y, w, h) in faces_rectangle:
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), thickness=2)
        face_locations += str(x) + ", " + str(y)

    cv.imshow('Detection', img)
    # rescaled = rescale_frame(img)
    # cv.imshow('Detected', rescaled)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
