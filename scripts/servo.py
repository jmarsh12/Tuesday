
import rospy
from std_msgs.msg import String
from dynamixel_sdk import *
import time

ADDR_PRO_TORQUE_ENABLE      = 24               
ADDR_PRO_GOAL_POSITION      = 30
ADDR_PRO_PRESENT_POSITION   = 37
PROTOCOL_VERSION            = 2.0       
DXL_ID                      = 2         
BAUDRATE                    = 57600     
DEVICENAME                  = '/dev/ttyUSB0'    
TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0             
DXL_MINIMUM_POSITION_VALUE  = 10           
DXL_MAXIMUM_POSITION_VALUE  = 4000         
DXL_MOVING_STATUS_THRESHOLD = 20           
portHandler = "global"
packetHander = "global"

def openPort():
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    if portHandler.openPort():
        print("Succeeded in opening port")

    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    return portHandler, packetHandler

def moveServo(portHandler, packetHandler, number, position):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, number, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, number, ADDR_PRO_GOAL_POSITION, position)
    if dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    time.sleep(1)
    
def closeServo(portHandler, packetHandler, number):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, number, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def closePort(portHandler, packetHandler):
    portHandler.closePort()
    
def callback(data):
    print(rospy.get_caller_id() + 'I heard %s', data.data)
    # Check for need to split multiple values
    # Data can be formatted"1,200" or "(1,200),(2,400)"
    pairs = data.data.split('),')

    for pair in pairs:
        pair = pair.replace('(', '') # Remove 
        pair = pair.replace(')', '') # Remove final )
        values = pair.split(',')
        servo = int(values[0])
        position  = int(values[1]) 
        moveServo(portHandler, packetHandler, servo, position)
        time.sleep(1)
    
    # BACKUP BELOW - If anything breaks, recover this code and delete the broken code
    #values = data.data.split(',')
    #servo = int(values[0])
    #position  = int(values[1]) 
    #moveServo(portHandler, packetHandler, servo, position)
    #time.sleep(2)

def listener():

    rospy.init_node('servo', anonymous = True)
    rospy.Subscriber('servo_control', String, callback)
    rospy.spin()


if __name__ == '__main__':
    
    portHandler, packetHandler = openPort()
    print("Servo is ready")
    listener()

    closeServo(portHandler, packetHandler, 4)
    closeServo(portHandler, packetHandler, 3)
    closePort(portHandler, packetHandler)