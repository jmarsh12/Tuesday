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
    
servo_publisher = None

def receive_coords(data):
    """
    data  - Int16MultiArray: [x, y]  or  String: "x, y"
    """
    # Receive img coords from opencv_find as Int16MultiArray
    # rospy.Publisher(str, Int16MultiArray, queue_size=1)
    # variable = Int16MulitArray

    coords = data.data # [x, y]  or  "x, y"

    # keep variables in scope
    x = 0.0
    y = 0.0
    # If String
    if isinstance(coords, str):
        x, y = coords.split(', ')
        x = float(x)
        y = float(y)
    # If Int16MultiArray
    else:
        x = coords[0]
        y = coords[1]

    # translate coords for servos
    servo_base_min = 200
    servo_base_max = 700
    
    new_x = int((servo_base_max - servo_base_min) * x + (servo_base_min))
    new_y = None # Not used yet
    servo_num = 7 # Just move the base during testing...
    
    # if publishing as int16MultiArray...
    # new_coords = [new_x, new_y]

    # if publishing as String...
    new_coords = str(servo_num) + ',' + str(new_x)
    
    # send new coords to servos
    servo_publisher.publish(new_coords) 

# TODO: Call this before anything else
def listener():

    # create rospy node and subscribe to finger coordinates
    rospy.init_node('move_arm', anonymous = True)
    rospy.Subscriber('opencv_coordinates', String, callback=receive_coords)
    servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    
    print("Move_Arm is ready")
    listener()