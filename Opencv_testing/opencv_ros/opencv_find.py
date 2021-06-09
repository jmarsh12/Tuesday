# import rospy
# from std_msgs.msg import String

from opencv_hand_tracker import *
import cv2



def main():
    # FPS times
    previousTime = 0
    currentTime = 0

    cap = cv2.VideoCapture(0)
    # Start: publish camera dimensions (x, y)

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

main()