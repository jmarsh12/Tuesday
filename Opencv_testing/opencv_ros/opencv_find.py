import cv2
import mediapipe as mp
import time



class hand_detector():
    """All of these parameters will be passed to Hands
        static_image_mode=False,         slower when true
        max_num_hands=2,                 max num hands to track
        min_detection_confidence=0.5, 
        min_tracking_confidence=0.5
    """
    def __init__(self, mode=False, max_hands=2, detection_confidence=0.5,track_confidence=0.5):
        self.mode = mode
        self.max_hands = max_hands
        self.detection_con = detection_confidence
        self.track_con = track_confidence

        # modules for hand tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(self.mode, self.max_hands, self.detection_con, self.track_con)
        # modules for drawing tracked points
        self.mpDraw = mp.solutions.drawing_utils

    def find_hands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Process the img
        self.results =self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            if draw:
                for handLms in self.results.multi_hand_landmarks:
                    # Draw hand landmarks
                    self.mpDraw.draw_landmarks(img, handLms, self.mp_hands.HAND_CONNECTIONS)

        return img

    def find_position(self, img, hand_num=0, draw=True):
        """
        img
        hand_hum    The index of the hand to be found
        draw            Do you want to draw on the image
        
        vocab - lm = landmark
                    id = index of landmark in hand. Each hand has 20.
        """

        lm_list = []
        
        if self.results.multi_hand_landmarks:
            my_hand = self.results.multi_hand_landmarks[hand_num]
            for id, lm in enumerate(my_hand.landmark):
                # convert from ratio coordinates to pixel coordinates
                # height, width, channel(rgb)
                h,w,c = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h) # ex. if x is 0.5 and the img is 500px wide, then 0.5 * 500px = 250px. Coordinate x is 250px

                lm_list.append([id, cx, cy])

                if draw:
                    cv2.circle(img, (cx, cy), 10, (255, 255, 0), cv2.FILLED)
            
        return lm_list


def main():
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
        detector.find_position(img, draw=False)

        # Calculate fps
        currentTime = time.time()
        fps = 1/(currentTime-previousTime)
        previousTime = currentTime
        # Display FPS
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)


        # Show the image
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()