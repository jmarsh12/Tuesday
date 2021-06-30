##################################################
# move_arm.py
# calculates the position the arm should move to after receiving the position of 
# an object, usually from opencv_find.py
# Publishes to servo_control
##################################################

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
from math import sin, cos, tan, asin, acos, atan, sqrt # import sin, cosine, and tangent
    
servo_publisher = None

# Servo Positions
STRAIGHT = 520 # arm facing straight forward
LEFT = 840
RIGHT = 220

# found at https://stackoverflow.com/questions/5294955/how-to-scale-down-a-range-of-numbers-with-a-known-min-and-max-value
def scale_between(unscaled_num, new_min, new_max, old_min, old_max):
  return (new_max - new_min) * (unscaled_num - old_min) / (old_max - old_min) + new_min


# gets the lower angle of a triangle and converts it to servo positions. 
# sides measured in pixels
def calc_base_pos(adjacent, hypotenuse):
    # calculate angle
    angle = acos(adjacent / hypotenuse)

    # convert to servo position
    servo_pos = scale_between(angle, 0, 90, STRAIGHT, LEFT)

    return servo_pos


def calc_hypotenuse(adjacent, opposite):
    hypotenuse = sqrt((adjacent^2 + opposite^2))
    return hypotenuse

def receive_coords(data):
    """
    data  - Int16MultiArray: [x, y]  or  String: "x, y"
    """
    # Receive img coords from opencv_find as Int16MultiArray
    # rospy.Publisher(str, Int16MultiArray, queue_size=1)
    # variable = Int16MulitArray

    coords = data.data # [x, y]  or  "x, y"

    # convert variable to usable values
    x = 0.0
    y = 0.0
    # if String
    if isinstance(coords, str):
        x, y = coords.split(', ')
        x = float(x)
        y = float(y)
    # If Int16MultiArray
    else:
        x = coords[0]
        y = coords[1]


    ##### translate coords for servos######
    base_servo = 7 # Just move the base during testing...
    # calculate base triangle
    
    side = 720
    base = 1280
    base_video_offset = 205# the space between the base of the video and the arm
    triangle_base = base / 2
    triangle_side = side + base_video_offset
    hypotenuse = calc_hypotenuse(triangle_base, triangle_side) # arm is in middle, so calculate triangle from left to middle

    ## Base Servo - Aim at object ##
    base_pos = calc_base_pos(triangle_side, hypotenuse)
    
    # if publishing as int16MultiArray, send base_pos as [new_x, new_y]
    servo_publisher.publish(str(base_servo) + ',' + str(base_pos)) 

    ## TODO: Top sevos - reach for object ##



# TODO: Call this before anything else
def listener():

    # create rospy node and subscribe to finger coordinates
    rospy.init_node('move_arm', anonymous = True)
    rospy.Subscriber('opencv_coordinates', String, callback=receive_coords)
    servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)

    # home arm
    servo_publisher.publish("(2,400),(3,500),(4,840),(5,500),(6,640),(7,450)") # one up from base

    rospy.spin()


if __name__ == '__main__':
    
    print("Move_Arm is ready")

    listener()