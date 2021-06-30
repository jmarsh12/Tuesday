##############################
# Servo.py
# Controls servos over a serial port
# each message can be sent like so:
# packetHandler.write4ByteTxRx(portHandler, number, parameter_num, parameter_val)
# Where parameter_num is the parameter you want to set (speed, position, torque, etc)
# and parameter_val is the value to set it to ( speed can be 0-2048, torque enable is 0 or 1)
# To get a list of parameters, see
# https://emanual.robotis.com/docs/en/dxl/x/xl320/
#------------------------------------------------------------------------------------
# Parameter_num															Min	Max
#		Size  Parameter Name		Description				Access	Initial value
#------------------------------------------------------------------------------------
# 24 	1 	Torque Enable			Motor Torque On/Off			RW 	0 	0 	1
# 25 	1 	LED						Status LED On/Off			RW 	0 	0 	7
# 27 	1 	D Gain					Derivative Gain				RW 	0 	0 	254
# 28 	1 	I Gain					Integral Gain				RW 	0 	0 	254
# 29 	1 	P Gain					Proportional Gain			RW 	32 	0 	254
# 30 	2 	Goal Position			Desired Position			RW 	- 	0 	1023
# 32 	2 	Moving Speed			Moving Speed		 		RW 	- 	0 	2047
# 35 	2 	Torque Limit			Torque Limit 				RW 	- 	0 	1023
# 37 	2 	Present Position		Present Position 			R 	- 	- 	-
# 39 	2 	Present Speed			Present Speed 				R 	- 	- 	-
# 41 	2 	Present Load			Present Load 				R 	- 	- 	-
# 45 	1 	Present Voltage			Present Voltage 			R 	- 	- 	-
# 46 	1 	Present Temperature 	Present Temperature 		R 	- 	- 	-
# 47 	1 	Registered Instruction  If Instruction is registeredR 	0 	- 	-
# 49 	1 	Moving					Movement Status 			R 	0 	- 	-
# 50 	1 	Hardware Error Status	Hardware Error Status 		R 	0 	- 	-
# 51 	2 	Punch					Minimum Current	Threshold 	RW 	32 	0 	1023

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
        pair = pair.replace('(', '') # Remove first (
        pair = pair.replace(')', '') # Remove final )
        values = pair.split(',')
        servo = int(values[0])
        position  = int(values[1]) 
        moveServo(portHandler, packetHandler, servo, position)
        time.sleep(1)

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
