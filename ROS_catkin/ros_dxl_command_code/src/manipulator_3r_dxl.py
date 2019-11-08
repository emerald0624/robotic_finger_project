#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import math
from dynamixel_sdk import *
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from msgpkg.msg import msgmag2
M_PI = 3.14159265358979323846
mag_en = 0.0
t_pre = 0.0
hz = 1./100
dxl_present_position=0

k_sc = 0.55
l_length = 0.058
# Control table address
ADDR_PRO_RETURN_TIME	    = 9
ADDR_PRO_OPERATING_MODE	    = 11
ADDR_PRO_HOMING_OFFSET 	    = 20
ADDR_PRO_MAX_POS 	    = 48
ADDR_PRO_MIN_POS 	    = 52
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
DXL1_ID                      = 1               # Dynamixel#1 ID : 2
DXL2_ID                      = 2               # Dynamixel#1 ID : 2
DXL3_ID                      = 3               # Dynamixel#1 ID : 2
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB4'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# ** setserial /dev/ttyUSB0 low_latency - if you want more communication speed, write this in terminal   https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/80

theta_pre1 = 1877
theta_pre2 = 2550
theta_pre3 = 1963

r_p = 14.4

r_j1 = 17
r_j2 = 14.4
r_j3 = 9.2

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

'''
def callback(msgmag2): 
    global mag_en   
    mag_en = msgmag2.mag2
'''
def dxl_input(dxl_goal_position1,dxl_goal_position2,dxl_goal_position3):

    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position1)),
                             DXL_HIBYTE(DXL_LOWORD(dxl_goal_position1)),
                             DXL_LOBYTE(DXL_HIWORD(dxl_goal_position1)),
                             DXL_HIBYTE(DXL_HIWORD(dxl_goal_position1))]

    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position2)),
                             DXL_HIBYTE(DXL_LOWORD(dxl_goal_position2)),
                             DXL_LOBYTE(DXL_HIWORD(dxl_goal_position2)),
                             DXL_HIBYTE(DXL_HIWORD(dxl_goal_position2))]

    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position3)),
                             DXL_HIBYTE(DXL_LOWORD(dxl_goal_position3)),
                             DXL_LOBYTE(DXL_HIWORD(dxl_goal_position3)),
                             DXL_HIBYTE(DXL_HIWORD(dxl_goal_position3))]

    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
    dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
    dxl_comm_result = groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

def dxl_pos_ctrl():
    global t_pre
    #global dxl_present_position
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    rospy.init_node('listener', anonymous=True)
    dxl_goal_position = 2048
    #dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
    #dxl_present_position[i], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    
    dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
    dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)
    dxl_comm_result = groupSyncRead.txRxPacket()
    dxl_present_position_1 = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    dxl_present_position_2 = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    dxl_present_position_3 = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    #print(dxl_present_position_1)
    #print(dxl_present_position_2)
    #print(dxl_present_position_3)

    t_pre = time.time()
    t0 = time.time()
    while not rospy.is_shutdown():
        cur = time.time()
        #rospy.Subscriber("magnetic_angle2", msgmag2, callback, queue_size = 1)
        if (cur - t_pre)>hz:
            t_cur = cur-t0
	    
	    '''
	    theta1 --> +
	    theta2 --> +
	    theta3 --> -
	    '''

	    #if t_cur<2:
		#q1_d = 0.0
	    #    q2_d = 0.0
	    #	q3_d = 0.0
            if t_cur<10:
		q1_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	        q2_d = 0.6*math.sin(M_PI*2*(t_cur-0.33)/2.0)
	    	q3_d = 0.6*math.sin(M_PI*2*(t_cur-0.66)/2.0)
	    elif t_cur<18:
		q1_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	        q2_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	    	q3_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	    elif t_cur<26:
		q1_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	        q2_d = 0.0
	    	q3_d = 0.0
	    elif t_cur<34:
		q1_d = 0.0
	        q2_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
            elif t_cur<42:
		q1_d = 0.0
	        q2_d = 0.0
	    	q3_d = 0.6*math.sin(M_PI*2*t_cur/2.0)
	    else:
		q1_d = 0.0
	        q2_d = 0.0
	    	q3_d = 0.0
		 
		
            #theta_des1 = 0.0  ### position (rad)
            #theta_des2 = 0.0  ### position (rad)
            #theta_des3 = 0.0  ### position (rad)
	    
            theta_des1 = (r_j1*q1_d)/r_p
            theta_des2 = (r_j2*q1_d + r_j2*q2_d)/r_p
            theta_des3 = -(r_j3*q1_d + r_j3*q2_d + r_j3*q3_d)/r_p
	    
	    #theta_des1 = 0.0
	    #theta_des1 = 0.0
	    #theta_des1 = 0.0
	    '''
            theta_des1 = 0.3*math.sin(M_PI*2*t_cur/10)  ### position (rad)
            theta_des2 = 0.3*math.sin(M_PI*2*t_cur/10)  ### position (rad)
            theta_des3 = 0.3*math.sin(M_PI*2*t_cur/10)  ### position (rad)
	    '''
            dxl_goal_position_1 = int(theta_des1 / M_PI * 2048 + theta_pre1)
            dxl_goal_position_2 = int(theta_des2 / M_PI * 2048 + theta_pre2)
            dxl_goal_position_3 = int(theta_des3 / M_PI * 2048 + theta_pre3)
            dxl_input(dxl_goal_position_1,dxl_goal_position_2,dxl_goal_position_3)

    	    dxl_comm_result = groupSyncRead.txRxPacket()
            dxl_present_position_1 = groupSyncRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            dxl_present_position_2 = groupSyncRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            dxl_present_position_3 = groupSyncRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
	    
 	    theta_cur = float(dxl_present_position_1-theta_pre1)/2048*M_PI
 	    theta_cur = float(dxl_present_position_1-theta_pre2)/2048*M_PI
 	    theta_cur = float(dxl_present_position_1-theta_pre3)/2048*M_PI
	    '''
    	    print(dxl_present_position_1)
    	    print(dxl_present_position_2)
    	    print(dxl_present_position_3)
	    '''
            t_pre = cur
	    '''
            if(abs(theta_des1)>0.5):
                dxl_goal_position_1 = 2002
                dxl_goal_position_2 = 2548
                dxl_goal_position_3 = 1966
                dxl_input(dxl_goal_position_1,dxl_goal_position_2,dxl_goal_position_3)
                break
            if(abs(theta_des2)>0.5):
                dxl_goal_position_1 = 2002
                dxl_goal_position_2 = 2548
                dxl_goal_position_3 = 1966
                dxl_input(dxl_goal_position_1,dxl_goal_position_2,dxl_goal_position_3)
                break            
	    if(abs(theta_des2)>0.5):
                dxl_goal_position_1 = 2002
                dxl_goal_position_2 = 2548
                dxl_goal_position_3 = 1966
                dxl_input(dxl_goal_position_1,dxl_goal_position_2,dxl_goal_position_3)
                break
	    '''

if __name__ == '__main__':
    try:
        t_pre = time.time()
        dxl_pos_ctrl()
    except rospy.ROSInterruptException:
        pass
