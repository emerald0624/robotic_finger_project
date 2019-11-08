#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import Float32
from msgpkg.msg import load_cell
from msgpkg.msg import fsr_sensor


ser = serial.Serial(port = '/dev/ttyUSB0',baudrate=115200)

def talker():
    pub1 = rospy.Publisher('Load_cell', load_cell, queue_size=1)
    pub2 = rospy.Publisher('fsr1', fsr_sensor, queue_size=1)
    pub3 = rospy.Publisher('fsr2', fsr_sensor, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1000) # 10hz
    while not rospy.is_shutdown():
        response = ser.readline()
        #rospy.loginfo(response)
        where1 = response.find(',')
	response2 = response[where1+1:]
        where2 = response2.find(',')
        va1 = float(response[:where1])
        va2 = float(response2[:where2])
        va3 = float(response2[where2+1:])
        
        #rospy.loginfo(va1)
        #rospy.loginfo(va2)
        pub1.publish(va1)
        pub2.publish(va2)
        pub3.publish(va3)
	#print(va3)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
