import rospy
from std_msgs.msg import String

from opencv_hand_tracker import *
import cv2

cap = cv2.VideoCapture(0)
detector = hand_detector()
opencv_publisher = None


# Receive message - object name
def find_object(data):
    """
    data  - an object holding String: object name

    Temporarily only tracks a hand and follows the pointer finger
    TODO: Change hand tracking for object finding
    """
    object_to_find = data.data
    # TODO: Search for object
    # for now, just spin looking for hands
    
    # FPS times
    previousTime = 0
    currentTime = 0

    cap = cv2.VideoCapture(0)
    detector = hand_detector()

    while True:

        # Take a photo
        success, img = cap.read()

        # search for and mark hands
        img = detector.find_hands(img)

        # get and display coordinates of one landmark
        ratio_coords = detector.get_coords_ratio(landmark_id=8, img=img, draw=True)
        
        # Make sure coordinates exist before writing them to the screen
        if coords:
            x, y = coords
            coord_disp = str(x) + ", " + str(y)
            cv2.putText(img, coord_disp, (10,120), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

        # Calculate fps
        currentTime = time.time()
        fps = 1/(currentTime-previousTime)
        previousTime = currentTime
        # Display FPS
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

        # Show the image
        cv2.imshow("Image", img)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

        sleep(0.2)
        opencv_publisher.publish(ratio_coords)


def listener():
    # create rospy node and subscribe to voice_commands
    rospy.init_node('opencv_find', anonymous = True)
    rospy.Subscriber('voice_commands', String, callback=find_object)
    opencv_publisher = rospy.Publisher('opencv_coordinates', String, queue_size=10)
    rospy.spin()

def track_and_display_hands():
    # FPS times
    previousTime = 0
    currentTime = 0

    cap = cv2.VideoCapture(0)
    detector = hand_detector()

    while True:
        # Take a photo
        success, img = cap.read()

        # search for and mark hands
        img = detector.find_hands(img)

        # get and display coordinates of one landmark
        coords = detector.get_coords_ratio(landmark_id=8, img=img, draw=True)
        
        if coords:
            x, y = coords
            coord_disp = str(x) + ", " + str(y)
            cv2.putText(img, coord_disp, (10,120), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)

        # Calculate fps
        currentTime = time.time()
        fps = 1/(currentTime-previousTime)
        previousTime = currentTime
        # Display FPS
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)


        # Show the image
        cv2.imshow("Image", img)
        cv2.waitKey(1)


def main():
    # track_and_display_hands()

    listener()

if __name__ == "__main__":
    main()