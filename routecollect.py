#!/usr/bin/env python
# license removed for brevity
import imp
import rospy
from std_msgs.msg import String
from car_control.msg import myGPS

import serial
import pandas as pd
import numpy as np
from math import sin,cos,pi

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
		GPSWeek.append(float(data2[1]))
		GPSTime.append(float(data2[2]))
		Heading.append(float(data2[3]))
		Pitch.append(float(data2[4]))
		Roll.append(float(data2[5]))
		Lattitude.append(float(data2[6])+(0.5*cos(pi/180*float(data2[3]))-0.1*sin(pi/180*float(data2[3])))/6356752*180/pi)
		Longitude.append(float(data2[7])+(0.5*sin(pi/180*float(data2[3]))+0.1*cos(pi/180*float(data2[3])))/6365986/cos(pi/180*Lattitude[-1])*180/pi)
		Altitude.append(float(data2[8]))
		Ve.append(float(data2[9]))
		Vn.append(float(data2[10]))
		Vu.append(float(data2[11]))
		Baseline.append(float(data2[12]))
		NSV1.append(float(data2[13]))
		NSV2.append(float(data2[14]))
		Status.append(str(data2[15]))

		La2.append(float(data2[6]))
		Lo2.append(float(data2[7])) 

		route_dic={'GPSTime':GPSTime,'Heading':Heading,'Pitch':Pitch,'Roll':Roll,'Lattitude':Lattitude,'Longitude':Longitude,'Altitude':Altitude,'Ve':Ve,'Vn':Vn,'Vu':Vu,'Baseline':Baseline,'NSV1':NSV1,'NSV2':NSV2,'Status':Status,'La2':La2,'Lo2':Lo2}
		route=pd.DataFrame(route_dic)
		route.to_csv('route.csv')
		
		GPSdata.GPSWeek=GPSWeek[-1]
		GPSdata.GPSTime=GPSTime[-1]
		GPSdata.Heading=Heading[-1]
		GPSdata.Pitch=Pitch[-1]
		GPSdata.Roll=Roll[-1]
		GPSdata.Lattitude=Lattitude[-1]
		GPSdata.Longitude=Longitude[-1]
		GPSdata.Altitude=Altitude[-1]
		GPSdata.Ve=Ve[-1]
		GPSdata.Vn=Vn[-1]
		GPSdata.Vu=Vu[-1]
		GPSdata.Baseline=Baseline[-1]
		GPSdata.NSV1=NSV1[-1]
		GPSdata.NSV2=NSV2[-1]
		GPSdata.Status=Status[-1]
		
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
	La2=[]
	Lo2=[]
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
