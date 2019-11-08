#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import Float32
from msgpkg.msg import fabric

ser = serial.Serial(port = '/dev/ttyUSB3',baudrate=115200)

def talker():
    pub1 = rospy.Publisher('fabric', fabric, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1000) # 10hz
    while not rospy.is_shutdown():
        response = ser.readline()
        va1 = float(response)
        
        rospy.loginfo(va1)
        #rospy.loginfo(va2)
        pub1.publish(va1)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
