#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from car_control.msg import myGPS

from math import cos,sin,pi
import serial
import pandas as pd
import numpy as np

def talker():
	pub = rospy.Publisher('GPSdata', myGPS, queue_size=10)
	rospy.init_node('GPSpublisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		GPSdata=myGPS()
		
		getBytes=b''
		data = str(serial.readline())
		print(data)
		data2 = data.split(",")
		
		GPSdata.GPSWeek=float(data2[1])
		GPSdata.GPSTime=float(data2[2])
		GPSdata.Heading=float(data2[3])
		GPSdata.Pitch=float(data2[4])
		GPSdata.Roll=float(data2[5])
		GPSdata.Lattitude=(float(data2[6])+(0.5*cos(pi/180*float(data2[3]))-0.1*sin(pi/180*float(data2[3])))/6356752*180/pi)
		GPSdata.Longitude=(float(data2[7])+(0.5*sin(pi/180*float(data2[3]))+0.1*cos(pi/180*float(data2[3])))/6365986/cos(pi/180*GPSdata.Lattitude)*180/pi)
		GPSdata.Altitude=float(data2[8])
		GPSdata.Ve=float(data2[9])
		GPSdata.Vn=float(data2[10])
		GPSdata.Vu=float(data2[11])
		GPSdata.Baseline=float(data2[12])
		GPSdata.NSV1=float(data2[13])
		GPSdata.NSV2=float(data2[14])
		GPSdata.Status=str(data2[15])
		
		rospy.loginfo(GPSdata)
		pub.publish(GPSdata)
		rate.sleep()

if __name__ == '__main__':
	serial = serial.Serial('/dev/GPSdata','115200',rtscts=True, dsrdtr=True)
	GPSWeek=[]
	GPSTime=[]
	Heading=[] 
	Pitch=[]
	Roll=[]
	Lattitude=[]
	Longitude=[]
	Altitude=[]
	Ve=[]
	Vn=[]
	Vu=[]
	Baseline=[]
	NSV1=[]
	NSV2=[]
	Status=[]
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
