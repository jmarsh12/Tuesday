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
from math import sin, cos, tan, asin, acos, atan, degrees, sqrt # import sin, cosine, and tangent
    
servo_publisher = None

# Servo Positions
STRAIGHT = 520 # arm facing straight forward
LEFT = 840
RIGHT = 220

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
ARM_HEIGHT_OFFSET = -205

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

# for triangles other than right
def calc_angle_opposite_of_side(side, base, adjacent):
    angle = degrees(acos((adjacent * adjacent + base * base - side * side)/(2.0 * adjacent * base)))
    return angle

# calc_hypotenuse_right 
# calculates the hypotenuse of a right triangle
def calc_hypotenuse_right_triangle(adjacent, opposite):
    hypotenuse =  sqrt((adjacent**2 + opposite**2))
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

    home(servo_publisher)
    
    # convert variable to usable values
    object_x_pos = 0.0
    object_y_pos = 0.0
    if isinstance(coords, str): # if String
        object_x_pos, object_y_pos = coords.split(',')
        object_x_pos = float(object_x_pos)
        object_y_pos = float(object_y_pos)
    else:                      # If Int16MultiArray
        object_x_pos = coords[0]
        object_y_pos = coords[1]


    ##### Calculate Servo Positions ######

    # Calculate Base Servo Angle
    base_servo = 4 
    arm_x_pos = CAMERA_WIDTH / 2 # arm sits in middle
    arm_y_pos = CAMERA_HEIGHT + ARM_HEIGHT_OFFSET # arm sits a little past the video frame
    
    arm_to_obj_x = abs(object_x_pos - arm_x_pos) # if negative, the object is on the left when facing the arm
    arm_to_obj_y = abs(object_y_pos - arm_y_pos) # subtracting a negative number makes the triangle bigger. Arm sits outside of square, so its offset is negative.

    # create sides of the triangle that will be used to get the angle
    arm_to_object_len = calc_hypotenuse_right_triangle(arm_to_obj_x, arm_to_obj_y)
    vert_triangle_hypotenuse = calc_hypotenuse_right_triangle(object_x_pos, arm_to_obj_y)

    ##  Aim arm at object - Base (rotating) Servo ##
    angle_arm_to_object = calc_angle_opposite_of_hypotenuse(arm_to_object_len, vert_triangle_hypotenuse, arm_x_pos )

    # convert degrees to base servo position
    base_pos = scale_between(angle_arm_to_object, 0, 180, LEFT, RIGHT)
    base_pos = int(base_pos)

    ## Reach for Object ##
    shoulder_servo = 5 #Hip (arm base bending) Servo
    wrist_twist_servo = 4
    wrist_bend_servo = 3
    elbow_servo = 6
    
    WRIST_LEN = 100 # TODO - add a real value. Measured in pixels
    WRIST_HEIGHT = 100 # TODO: measure this, too.
    SHOULDER_HEIGHT = 50 # TODO: measure this, too.  
    SERVO_LEN = 100 # TODO: measure this, too.

    wrist_bend_to_shoulder_width = arm_to_object_len - WRIST_LEN
    wrist_bend_to_shoulder_height = WRIST_HEIGHT - SHOULDER_HEIGHT
    wrist_bend_to_shoulder_hypotenuse = calc_hypotenuse_right_triangle( wrist_bend_to_shoulder_width, wrist_bend_to_shoulder_height )
    elbow_angle = calc_angle_opposite_of_side(wrist_bend_to_shoulder_hypotenuse, SERVO_LEN, SERVO_LEN)
    wrist_angle = calc_angle_opposite_of_side(SERVO_LEN, wrist_bend_to_shoulder_hypotenuse, SERVO_LEN)
    shoulder_angle = wrist_angle

    # TODO: Send angles to servos

    # if publishing as int16MultiArray, send base_pos as [new_x, new_y]
    servo_publisher.publish(str(base_servo) + ',' + str(base_pos)) 
    time.sleep(0.3)
    servo_publisher.publish(str(base_servo) + ',' + str(base_pos))

    ## TODO: Top servos - reach for object ##

def home(servo_publisher):
  servo_publisher.publish("(2,400),(3,500),(4,840),(5,500),(6,640),(7,450)") # one up from base

# TODO: Call this before anything else
def listener():

    # create rospy node and subscribe to finger coordinates
    rospy.init_node('move_arm', anonymous = True)
    servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)
    # rospy.Subscriber('opencv_coordinates', String, callback=test)
    rospy.Subscriber('opencv_coordinates', String, callback=receive_coords)
    

    # TODO: home arm
    

    print("Move arm is running")
    rospy.spin()


if __name__ == '__main__':

    listener()

    print("Move_Arm is ready")
