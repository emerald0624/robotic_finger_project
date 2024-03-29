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
import time
import serial
import time
import os
from std_msgs.msg import Float32
from msgpkg.msg import msgmag
from msgpkg.msg import msgmag2

ser = serial.Serial(port = '/dev/ttyUSB3',baudrate=115200)

pre_value1 = 0
pre_value2 = 0
pre_value3 = 0
pre_value12 = 0
pre_value22 = 0
pre_value32 = 0
def ADC2ANG1(adc):
    global pre_value1, pre_value2, pre_value3
    ANG = 2.0*3.1415926536*float(adc)/(16383.0)
    summation = (ANG+pre_value1+pre_value2+pre_value3)/4.00
    pre_value1 = ANG
    pre_value2 = pre_value1
    pre_value3 = pre_value2
    return summation

def ADC2ANG2(adc):
    global pre_value12, pre_value22, pre_value32
    ANG = 2.0*3.1415926536*float(adc)/(16383.0)
    summation = (ANG+pre_value12+pre_value22+pre_value32)/4.00
    pre_value12 = ANG
    pre_value22 = pre_value12
    pre_value32 = pre_value22
    return summation

def magnetic_read():
    pub = rospy.Publisher('magnetic_angle', msgmag, queue_size=1)

    rospy.init_node('magnetic_read', anonymous=True)
    ser.write('a')	
    rate = rospy.Rate(5000) # 100hz
    time.sleep(0.01)
    for i in range(3):
        response = ser.readline()
        where = response.find(',')
        value = ADC2ANG1(int(response[:where]))

    
    #pre = time.time()
    hz = 1./550
    while not rospy.is_shutdown():
        #cur = time.time()
        response = ser.readline()
        where = response.find(',')
        value = ADC2ANG1(int(response))

        #value = float(ser.readline())
        #    value = float(ser.readline())
        #if (cur-pre)>hz:
        #    value = float(ser.readline())
        #    rospy.loginfo(response)
        #    pub.publish(value)
        #    pub2.publish(value2)
        #    pre = cur
        rospy.loginfo(value)
        pub.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        magnetic_read()
    except rospy.ROSInterruptException:
        pass
