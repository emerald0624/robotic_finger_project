#!/usr/bin/env python
import rospy
import time
import serial
from msgpkg.msg import load_cell

M_PI = 3.14159265358979323846
hz = 100.0
M_G = 9.810616

# magnetic encoder

ser = serial.Serial(port = '/dev/ttyUSB1',baudrate=115200)

def talker():
    pub1 = rospy.Publisher('load_cell1', load_cell, queue_size=1)
    pub2 = rospy.Publisher('load_cell2', load_cell, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    t0 = time.time()
    t_pre = time.time()
    while not rospy.is_shutdown():
	t_cur = time.time()
	if (t_cur-t_pre)>1/float(hz):
	    #### read
	    response = ser.readline()
	    where1 = response.find(',')	
	    va1 = float(response[:where1])
	    va2 = float(response[where1+1:])

	    #### data
	    load1 = float(va1)/1000.0*M_G
	    load2 = float(va2)/1000.0*M_G

	    #### pub
	    pub1.publish(load1)
	    pub2.publish(load2)
	    #print(load1)
	    
	    t_pre = t_cur;

	    #response = ser.readline()
            #where1 = response.find(',')
	    #response2 = response[where1+1:]
            #where2 = response2.find(',')
            #va1 = float(response[:where1])
            #va2 = float(response2[:where2])
            #va3 = float(response2[where2+1:])
            #pub1.publish(va1)
            #pub2.publish(va2)
            #pub3.publish(va3)
            #rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


