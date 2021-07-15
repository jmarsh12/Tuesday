##################################################
# move_arm.py
# calculates the position the arm should move to after receiving the position of 
# an object, usually from opencv_find.py
# Publishes to servo_control
##################################################

#################################################
# Positions:
#                             
#                  500
#            375    |   700
#               \  ___ /
# Backward       /  ^  \
#        200 -- |  360  | -- 825   Forward
#                \ ___ /
#               /       \ 
#              0       1023
#
# Droop from servo: compensate by about 50 (ex. 200 should be 250 to get a
# 90 degree angle on the bottom, where the base hits 90 degrees at 200
#
# Mouth:
#   Full open:     250
#   Halfway:       450
#   Full closed:   600
#   Pressure hold: 700
#   
# Servos:
#    3          Wrist 2  - rotates
#    #-#   7    Hand     - bends
#    #   6      Wrist 1  - bends
#    #   5      Elbow    - bends
#    #   2      Shoulder - bends
#   _#_  4      Base     - rotates  hip to ground: 
#
#################################################


import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
from math import sin, cos, tan, asin, acos, atan, degrees, sqrt # import sin, cosine, and tangent

# Servo numbers
BASE = 4
SHOULDER = 2
ELBOW = 5
WRIST_1 = 6
WRIST_2 = 3
HAND = 7

FRAME_WIDTH = 640 # 1280
FRAME_HEIGHT = 480 # 720

# Measurements
# get this by placing a ruler vertically in the camera view
CAMERA_VIEW_HEIGHT_MM = 146

PIXELS_PER_MM = FRAME_HEIGHT / CAMERA_VIEW_HEIGHT_MM
TOTAL_HEIGHT = 225 * PIXELS_PER_MM
# Difference between the height of wrist1 and the hip
WRIST1_TO_GROUND = 114 * PIXELS_PER_MM
WRIST1_TO_WRIST2 = 60 * PIXELS_PER_MM
ELBOW_TO_WRIST1 = 55 * PIXELS_PER_MM
SHOULDER_TO_ELBOW = 55 * PIXELS_PER_MM
SHOULDER_TO_GROUND = 50 * PIXELS_PER_MM
ARM_HEIGHT_OFFSET = -40 * PIXELS_PER_MM
# wrist1 to mid angle.
WRIST1_TO_MID = WRIST1_TO_GROUND - SHOULDER_TO_GROUND

# Base Servo Positions
STRAIGHT = 520 # arm facing straight forward
LEFT = 840
RIGHT = 220

# Other servos
SERVO_180 = 500
SERVO_0 = SERVO_180 - 615 # This position can't be reached because the arm can't bend into itself - hence position -115

# The arm's position in the frame (it sits a little outside)
ARM_X_POS = FRAME_WIDTH / 2 # arm sits in middle
ARM_Y_POS = ARM_HEIGHT_OFFSET # arm sits a little past the video frame

servo_publisher = None

# found at https://stackoverflow.com/questions/5294955/how-to-scale-down-a-range-of-numbers-with-a-known-min-and-max-value
def scale_between(unscaled_num, old_min, old_max, new_min, new_max):
  return (new_max - new_min) * (unscaled_num - old_min) / (old_max - old_min) + new_min


# gets the lower angle of a triangle and converts it to servo positions. 
# sides measured in pixels
def calc_base_pos(adjacent, hypotenuse):
    # calculate angle
    angle = acos(adjacent / hypotenuse)

    # convert to servo position
    servo_pos = scale_between(angle, 0, 90, STRAIGHT, LEFT)

    return servo_pos

# Calculate an angle of a non-right triangle
def calc_angle_opposite_of_side(opposite, A, B):
    print("Side 1: "+ str(opposite))
    print("Side 2: " + str(A))
    print("Side 3: "+ str(B))

    angle = degrees(acos((A * A + B * B - opposite * opposite)/(2.0 * A * B)))
    print("Final Angle: ", angle)
    return angle

# calc_hypotenuse_right 
# calculates the hypotenuse of a right triangle
def calc_hypotenuse_right_triangle(adjacent, opposite):
    hypotenuse =  sqrt(adjacent**2 + opposite**2)
    return hypotenuse

def receive_coords(data):
    """
    data  - Int16MultiArray: [x, y]  or  String: "x, y"
    """
    # Receive img coords from opencv_find as Int16MultiArray
    # rospy.Publisher(str, Int16MultiArray, queue_size=1)
    # variable = Int16MulitArray
    #
    #           |  | - arm base
    # 0 1 2 3 4 5 6 7 8 9
    # 1 0 0 0 0 0 0 0 0 0
    # 2 0 0 0 0 0 0 0 0 0
    # 3 0 0 0 0 0 0 0 0 0
    # 4 0 0 0 0 0 0 0 0 0
    # 5 0 0 0 0 0 0 0 0 0
    #           | | - camera stand

    print("Received coords: ", data.data)
    
    servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)
    coords = data.data # [x, y]  or  "x, y"
    
    # convert variable to usable values
    object_x_pos = None
    object_y_pos = None
    if isinstance(coords, str): # if String
        object_x_pos, object_y_pos = coords.split(',')
        object_x_pos = float(object_x_pos)
        object_y_pos = float(object_y_pos)
    else:                      # If Int16MultiArray
        object_x_pos = coords[0]
        object_y_pos = coords[1]


    ##### Calculate Servo Positions ######

    ### Base ###
    
    object_x_pos = 638
    object_y_pos = 50
    # find distance between object and base
    arm_to_obj_x = object_x_pos - ARM_X_POS # if negative, the object is on the left when facing the arm

    arm_to_obj_y = object_y_pos - ARM_Y_POS # expand the triangle down to the arm
    
    base_to_object_len = sqrt(abs(arm_to_obj_x)**2 + arm_to_obj_y**2)

    # Find angle of base
    base_angle = degrees(atan(arm_to_obj_y / abs(arm_to_obj_x)))

    if arm_to_obj_x >= 0: # if the object sits on the right side of the screen
        base_angle += 90

    # convert angle to position
    base_pos = int(scale_between(base_angle, 0, 180, RIGHT, LEFT))
    # NOTE: debugging print statement 
    # print(f"Base angle: {base_angle}\nBase Position: {base_pos}\n")

    servo_publisher.publish(str(BASE) + ',' + str(base_pos))

    ### Upper Arm ###
    #    
    #   /#--#
    # #      #
    # |      |\
    # #      | \

    wrist_bend_to_shoulder_width = base_to_object_len - WRIST1_TO_WRIST2
    wrist_bend_to_shoulder_hypotenuse = sqrt( wrist_bend_to_shoulder_width**2 + WRIST1_TO_MID**2 )
    # elbow_angle = calc_angle_opposite_of_side(wrist_bend_to_shoulder_hypotenuse, SHOULDER_TO_ELBOW, ELBOW_TO_WRIST1)
    # wrist_angle = calc_angle_opposite_of_side(SHOULDER_TO_ELBOW, wrist_bend_to_shoulder_hypotenuse, ELBOW_TO_WRIST1)
    # shoulder_angle = wrist_angle
    # print("Elbow Angle: ", elbow_angle)
    # print("Wrist 1 Angle: ", wrist_angle)
    # print("Shoulder Angle: ", shoulder_angle)



def home_arm(servo_publisher):
  servo_publisher.publish("(2,400),(3,500),(4,840),(5,500),(6,640),(7,450)") # one up from base

# Call this before anything else
def listener():

    # create rospy node
    rospy.init_node('move_arm', anonymous = True)
    servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)

    home_arm(servo_publisher)

    # subscribe to opencv coordinates
    rospy.Subscriber('opencv_coordinates', String, callback=receive_coords)
    
    print("Move arm is running")
    rospy.spin()


if __name__ == '__main__':

    listener()

    print("Move_Arm is ready")
