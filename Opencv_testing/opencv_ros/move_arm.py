import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import time
    
# May not work until after init_node has been run
servo_publisher = rospy.Publisher('servo_control', String, queue_size=10)

def callback(data):
    print("Feelin' good")

def receive_coords(data):
    # Receive img coords from opencv_find as Int16MultiArray
    coords = data.data # [x, y]  or  "x, y"
    x = 0.0
    y = 0.0
    # If it's a string...
    if isinstance(coords, str):
        x, y = coords.split(', ')
        x = float(x)
        y = float(y)
    else:
        # If it's an array...
        x = coords[0]
        y = coords[1]

    # translate coords for servos
    servo_base_min = 200
    servo_base_max = 700
    
    new_x = int((servo_base_max - servo_base_min) * x + (servo_base_min))
    new_y = 15 # just because...
    
    # if publishing as int16MultiArray...
    # new_coords = [new_x, new_y]

    # if publishing as String...
    new_coords = str(new_x) + ',' + str(new_y)

    # send new coords to servos
    servo_publisher.publish(new_coords) 

def listener():

    rospy.init_node('move_arm', anonymous = True)
    rospy.Subscriber('finger_coords', String, callback=receive_coords)
    rospy.spin()


if __name__ == '__main__':
    
    print("Move_Arm is ready")
    listener()