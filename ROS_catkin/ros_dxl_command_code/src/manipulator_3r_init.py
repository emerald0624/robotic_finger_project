#!/usr/bin/env python

import rospy
import time
from dynamixel_sdk import *
from std_msgs.msg import Int32

# Control table address
ADDR_PRO_BAUD_RATE	    	= 8
ADDR_PRO_RETURN_TIME	    = 9
ADDR_PRO_OPERATING_MODE	    = 11
ADDR_PRO_HOMING_OFFSET 	    = 20
ADDR_PRO_POS_P_GAIN			= 84
ADDR_PRO_POS_I_GAIN			= 82
ADDR_PRO_POS_D_GAIN			= 80
ADDR_PRO_FEEDFORWARD_FIRST_GAIN		= 90
ADDR_PRO_FEEDFORWARD_SECOND_GAIN	= 88
ADDR_PRO_MAX_POS 	    	= 48
ADDR_PRO_MIN_POS 	    	= 52
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_PROFILE_VELOCITY   = 112
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4
LEN_PRO_PROFILE_VELOCITY    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1               # Dynamixel#1 ID : 1
DXL2_ID                      = 2               # Dynamixel#2 ID : 2
DXL3_ID                      = 3               # Dynamixel#3 ID : 3

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 115200
DEVICENAME                  = '/dev/ttyUSB4'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
BAUD_RATE_SET		    	= 3
OPERATING_MODE		    	= 3
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0          	# Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4095            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 1                # Dynamixel moving status threshold
DXL_RETURN_DELAY_TIME       = 10		# Dynamixel Return delay time (0*2us)

# PID gain
# PID gain #1
DXL1_POS_P_GAIN			= 2000
DXL1_POS_I_GAIN			= 0
DXL1_POS_D_GAIN			= 0
DXL1_POS_FEEDFORWARD_FIRST_GAIN  = 0
DXL1_POS_FEEDFORWARD_SECOND_GAIN = 0
# PID gain #2
DXL2_POS_P_GAIN			= 2000
DXL2_POS_I_GAIN			= 0
DXL2_POS_D_GAIN			= 0
DXL2_POS_FEEDFORWARD_FIRST_GAIN  = 0
DXL2_POS_FEEDFORWARD_SECOND_GAIN = 0
# PID gain #3
DXL3_POS_P_GAIN			= 2000
DXL3_POS_I_GAIN			= 0
DXL3_POS_D_GAIN			= 0
DXL3_POS_FEEDFORWARD_FIRST_GAIN  = 0
DXL3_POS_FEEDFORWARD_SECOND_GAIN = 0
# index = 0
# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize Groupsyncwrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize Groupsyncread Structs for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)



def dxl_init():
    # Open Port
    if portHandler.openPort():
        print "Succeeded to open the port"
    else:
        print "Failed to open the port"
        quit()

    # Set Port Baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print "Succeeded to change the Baudrate"
    else:
        print "Failed to change the Baudrate"
        quit()


    #unable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#1 is disable to torque"

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#2 is disable to torque"

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#3 is disable to torque"


    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_BAUD_RATE, BAUD_RATE_SET)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Baudrate of Dynamixel#1 is 1000000"

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_BAUD_RATE, BAUD_RATE_SET)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Baudrate of Dynamixel#2 is 1000000"

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_BAUD_RATE, BAUD_RATE_SET)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Baudrate of Dynamixel#3 is 1000000"

    #Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Set Position Mode of Dynamixel#1"

    #Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Set Position Mode of Dynamixel#2"

    #Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, OPERATING_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Set Position Mode of Dynamixel#3"

    # DXL1
    #DXL_P_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POS_P_GAIN, DXL1_POS_P_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS p gain of Dynamixel#1 has been successfully changed : %04d" %(DXL1_POS_P_GAIN)
  
    #DXL_I_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POS_I_GAIN, DXL1_POS_I_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS i gain of Dynamixel#1 has been successfully changed : %04d" %(DXL1_POS_I_GAIN)

    #DXL_D_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POS_D_GAIN, DXL1_POS_D_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS d gain of Dynamixel#1 has been successfully changed : %04d" %(DXL1_POS_D_GAIN)

    #DXL_FEEDFORWARD_1st_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_FIRST_GAIN, DXL1_POS_FEEDFORWARD_FIRST_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 1st gain of Dynamixel#1 has been successfully changed : %04d" %(DXL1_POS_FEEDFORWARD_FIRST_GAIN)

    #DXL_FEEDFORWARD_2nd_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_SECOND_GAIN, DXL1_POS_FEEDFORWARD_SECOND_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 2st gain of Dynamixel#1 has been successfully changed : %04d" %(DXL1_POS_FEEDFORWARD_SECOND_GAIN)

    # DXL2
    # DXL_P_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POS_P_GAIN, DXL2_POS_P_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS p gain of Dynamixel#2 has been successfully changed : %04d" % (DXL2_POS_P_GAIN)

    # DXL_I_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POS_I_GAIN,
                                                              DXL2_POS_I_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS i gain of Dynamixel#2 has been successfully changed : %04d" % (DXL2_POS_I_GAIN)

    # DXL_D_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POS_D_GAIN,
                                                              DXL2_POS_D_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS d gain of Dynamixel#2 has been successfully changed : %04d" % (DXL2_POS_D_GAIN)

    # DXL_FEEDFORWARD_1st_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_FIRST_GAIN,
                                                              DXL2_POS_FEEDFORWARD_FIRST_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 1st gain of Dynamixel#2 has been successfully changed : %04d" % (DXL2_POS_FEEDFORWARD_FIRST_GAIN)

    # DXL_FEEDFORWARD_2nd_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_SECOND_GAIN,
                                                              DXL2_POS_FEEDFORWARD_SECOND_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 2st gain of Dynamixel#2 has been successfully changed : %04d" % (DXL2_POS_FEEDFORWARD_SECOND_GAIN)


    # DXL3
    # DXL_P_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POS_P_GAIN, DXL3_POS_P_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS p gain of Dynamixel#3 has been successfully changed : %04d" % (DXL3_POS_P_GAIN)

    # DXL_I_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POS_I_GAIN,
                                                              DXL3_POS_I_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS i gain of Dynamixel#3 has been successfully changed : %04d" % (DXL3_POS_I_GAIN)

    # DXL_D_GAIN_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_POS_D_GAIN,
                                                              DXL3_POS_D_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "POS d gain of Dynamixel#3 has been successfully changed : %04d" % (DXL3_POS_D_GAIN)

    # DXL_FEEDFORWARD_1st_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_FIRST_GAIN,
                                                              DXL3_POS_FEEDFORWARD_FIRST_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 1st gain of Dynamixel#3 has been successfully changed : %04d" % (DXL3_POS_FEEDFORWARD_FIRST_GAIN)

    # DXL_FEEDFORWARD_2nd_VALUE
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_FEEDFORWARD_SECOND_GAIN,
                                                              DXL3_POS_FEEDFORWARD_SECOND_GAIN)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "feedforward 2st gain of Dynamixel#3 has been successfully changed : %04d" % (DXL3_POS_FEEDFORWARD_SECOND_GAIN)


    #Return Delay Time 20us
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_RETURN_TIME, DXL_RETURN_DELAY_TIME)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Return delay time of Dynamixel#1 has been successfully changed : %02d" %(DXL_RETURN_DELAY_TIME*2)

    #Return Delay Time 20us
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_RETURN_TIME, DXL_RETURN_DELAY_TIME)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Return delay time of Dynamixel#2 has been successfully changed : %02d" %(DXL_RETURN_DELAY_TIME*2)

    #Return Delay Time 20us
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_RETURN_TIME, DXL_RETURN_DELAY_TIME)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Return delay time of Dynamixel#3 has been successfully changed : %02d" %(DXL_RETURN_DELAY_TIME*2)

    #DXL_MAXIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MAX_POS, DXL_MAXIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MAXIMUM POS of Dynamixel#1 has been successfully changed : %04d" %(DXL_MAXIMUM_POSITION_VALUE)

    #DXL_MAXIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MAX_POS, DXL_MAXIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MAXIMUM POS of Dynamixel#2 has been successfully changed : %04d" %(DXL_MAXIMUM_POSITION_VALUE)

    #DXL_MAXIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MAX_POS, DXL_MAXIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MAXIMUM POS of Dynamixel#3 has been successfully changed : %04d" %(DXL_MAXIMUM_POSITION_VALUE)

    #DXL_MINIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MIN_POS, DXL_MINIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MINIMUM POS of Dynamixel#1 has been successfully changed : %04d" %(DXL_MINIMUM_POSITION_VALUE)

    #DXL_MINIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MIN_POS, DXL_MINIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MINIMUM POS of Dynamixel#2 has been successfully changed : %04d" %(DXL_MINIMUM_POSITION_VALUE)

    #DXL_MINIMUM_POSITION_VALUE
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_MIN_POS, DXL_MINIMUM_POSITION_VALUE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "MINIMUM POS of Dynamixel#3 has been successfully changed : %04d" %(DXL_MINIMUM_POSITION_VALUE)

    #Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#1 is able to torque"

    #Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#2 is able to torque"

    #Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print "%s" % packetHandler.getTxRxResult(dxl_comm_result)
    elif dxl_error != 0:
        print "%s" % packetHandler.getRxPacketError(dxl_error)
    else:
        print "Dynamixel#3 is able to torque"

    # dxl_goal_position = 2048
    # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
    # dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    

if __name__ == '__main__':
    dxl_init()
    portHandler.closePort()

