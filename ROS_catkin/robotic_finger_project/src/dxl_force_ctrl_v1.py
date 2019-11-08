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
DXL_ID                      = 1               # Dynamixel#1 ID : 2
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB2'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# ** setserial /dev/ttyUSB0 low_latency - if you want more communication speed, write this in terminal   https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/80


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def callback(msgmag2): 
    global mag_en   
    mag_en = msgmag2.mag2


def callback(exo_tor): 
    global tor_exo   
    tor_exo = exo_tor.Data


def dxl_pos_read():    
    global t_pre
    global dxl_present_position
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    rospy.init_node('listener', anonymous=True)
    dxl_goal_position = 2048
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

    t_pre = time.time()
    pub = rospy.Publisher('angle_finger', Float32, queue_size=1)
    pub2 = rospy.Publisher('angle_dxl_des', Float32, queue_size=1)
    pub3 = rospy.Publisher('force_finger_des', Float32, queue_size=1)
    pub4 = rospy.Publisher('angle_dxl', Float32, queue_size=1)
    pub5 = rospy.Publisher('angle_defl', Float32, queue_size=1)
    cnt = 0.0
    t0 = time.time()
    while not rospy.is_shutdown():
	cur = time.time()
	rospy.Subscriber("magnetic_angle2", msgmag2, callback, queue_size = 1)
	rospy.Subscriber("exo_tor", exo_tor, callback2, queue_size = 1)
	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	if (cur - t_pre)>hz:
	    t_cur = cur-t0;
	    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	    '''
	    if t_cur< 10:
	   	f_des = 0
	    elif t_cur < 15:
		f_des = 0.5
	    elif t_cur < 20:
		f_des = 1.0
	    elif t_cur < 25:
		f_des = 1.5
	    elif t_cur < 30:
		f_des = 2.0
	    elif t_cur < 35:
		f_des = 1.5
	    elif t_cur < 40:
		f_des = 1.0
	    elif t_cur < 45:
		f_des = 0.5
	    else:
		f_des = 0.0
	    '''
	    '''
	    if t_cur<10:
	    	f_des = 0
	    elif t_cur<50:
		f_des = 2.0 * math.sin(2*M_PI*(t_cur-10)/10)
	    else : 
		f_des = 0
	    if f_des <0:
		f_des = 0
	    '''
	    #f_des = tor_exo*0.05
	    q_cur = float(dxl_present_position-2048)/2048*M_PI - mag_en
	    theta_des = q_cur + float(f_des*l_length)/k_sc
	    #print('des_motor : ',theta_des)
	    #print('motor : ',float(dxl_present_position-2048)/2048*M_PI)
	    print(theta_des-float(dxl_present_position-2048)/2048*M_PI)
	    dxl_goal_position = int(theta_des/M_PI*2048+2048)
	    pub.publish(q_cur)
	    pub2.publish(theta_des)
	    pub3.publish(f_des)
	    pub4.publish(float(dxl_present_position-2048)/2048*M_PI)
	    pub5.publish(mag_en)
	    #rospy.loginfo(theta_des)
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            #dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
	    t_pre = cur

	    if(abs(theta_des)>1.3):
		dxl_goal_position = 2048
		dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,ADDR_PRO_GOAL_POSITION, dxl_goal_position)
            	dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
		break


if __name__ == '__main__':
    try:
        t_pre = time.time()
        dxl_pos_read()
    except rospy.ROSInterruptException:
        pass
