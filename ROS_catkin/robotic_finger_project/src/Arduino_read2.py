#!/usr/bin/env python
import rospy
import time
import serial
from msgpkg.msg import magnetic_encoder
from msgpkg.msg import fsr

M_PI = 3.14159265358979323846
hz = 1000.0
n_init = 20

# magnetic encoder

ser = serial.Serial(port = '/dev/ttyUSB2',baudrate=115200)

def talker():
    pub1 = rospy.Publisher('sen1', fsr, queue_size=1)
    pub2 = rospy.Publisher('sen2', fsr, queue_size=1)
    pub3 = rospy.Publisher('sen3', fsr, queue_size=1)
    pub4 = rospy.Publisher('sen4', fsr, queue_size=1)
    pub5 = rospy.Publisher('sen5', fsr, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    t0 = time.time()
    t_pre = time.time()
    sum_1 = 0
    i=0
    while i < n_init:
	t_cur = time.time()
	if (t_cur-t_pre)>1/float(hz):
	    response = ser.readline()
	    #print(response)
	    
            where1 = response.find(',')
	    va1 = float(response[:where1])
	    response2 = response[where1+1:]

            where2 = response2.find(',')	    
	    va2 = float(response2[:where2])
	    response3 = response2[where2+1:]

            where3 = response3.find(',')	    
	    va3 = float(response3[:where3])
	    response4 = response3[where3+1:]

            where4 = response4.find(',')	    
	    va4 = float(response4[:where4])
	    response5 = response4[where4+1:]
	     
	    va5 = float(response5)
	    t_pre = t_cur
	    i = i+1
	    

    base_1 = float(sum_1)/float(n_init)
    t0 = time.time()
    t_pre = time.time()

    while not rospy.is_shutdown():
	t_cur = time.time()
	if (t_cur-t_pre)>1/float(hz):
	    #### read
	    response = ser.readline()            
	    
	    where1 = response.find(',')
	    va1 = float(response[:where1])
	    response2 = response[where1+1:]

            where2 = response2.find(',')	    
	    va2 = float(response2[:where2])
	    response3 = response2[where2+1:]

            where3 = response3.find(',')	    
	    va3 = float(response3[:where3])
	    response4 = response3[where3+1:]

            where4 = response4.find(',')	    
	    va4 = float(response4[:where4])
	    response5 = response4[where4+1:]
	     
	    va5 = float(response5)

	    #### pub
	    pub1.publish(va1)
	    pub2.publish(va2)
	    pub3.publish(va3)
	    pub4.publish(va4)
	    pub5.publish(va5)
	    

	    #print(angle_data_1,fsr_1)
	    
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


