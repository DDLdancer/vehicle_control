#!/usr/bin/env python
from turtle import heading
import rospy
from std_msgs.msg import String
from car_control.msg import myGPS
from geometry_msgs.msg import Twist

from math import sqrt, pow, acos, tan, radians, cos, pi, sin
import pandas as pd


def angle_of_vector(v1, v2):
    vector_prod = v1[0] * v2[0] + v1[1] * v2[1]
    length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * \
        sqrt(pow(v2[0], 2) + pow(v2[1], 2))
    cos_v = vector_prod * 1.0 / (length_prod * 1.0 + 1e-6)
    cross_prod = v1[0]*v2[1]-v1[1]*v2[0]
    if(cross_prod > 0):
        return (acos(cos_v) / pi) * 180
    else:
        return -(acos(cos_v) / pi) * 180


def control_to_point(xd, yd, Heading, A=0.5, B=0.005):
    v = min(A*(xd**2+yd**2), 0.4)
    vx = sin(Heading*pi/180)
    vy = cos(Heading*pi/180)
    if angle_of_vector([xd, yd], [vx, vy]) < 0:
        w = min(-B*angle_of_vector([xd, yd], [vx, vy]), 1)  # TODO
    else:
        w = max(-B*angle_of_vector([xd, yd], [vx, vy]), -1)  # TODO
    return v, w


def control_Heading(route_Heading, Heading, B=0.01):
    v = 0
    xd = sin(route_Heading*pi/180)
    yd = cos(route_Heading*pi/180)
    vx = sin(Heading*pi/180)
    vy = cos(Heading*pi/180)
    if abs(angle_of_vector([xd, yd], [vx, vy])) < 5:
        w = 0
        print("arrive!")
    else:
        if angle_of_vector([xd, yd], [vx, vy]) < 0:
            w = min(-B*angle_of_vector([xd, yd], [vx, vy]), 1)  # TODO
        else:
            w = max(-B*angle_of_vector([xd, yd], [vx, vy]), -1)  # TODO
    return v, w


def calcDistance(Lat_A, Lng_A, Lat_B, Lng_B):
    ra = 6378.137 * 1000
    rb = 6356.752 * 1000
    R = sqrt((1+tan(Lat_A)**2)/(1/ra**2+tan(Lat_A)**2/rb**2))
    rad_lat_A = radians(Lat_A)
    rad_lng_A = radians(Lng_A)
    rad_lat_B = radians(Lat_B)
    rad_lng_B = radians(Lng_B)
    xd = cos(rad_lat_A) * R * (rad_lng_B - rad_lng_A)
    yd = rb * (rad_lat_B - rad_lat_A)
    return xd, yd


def controlfunction(GPSdata):
    # route=loadroute()
    x = 0
    if GPSdata.Pitch > 20:
        x = 0.2

    if GPSdata.Pitch < -20:
        x = -0.2
    cmd = Twist()
    cmd.linear.x = x
    return cmd


def find_initiali(route):
    i = route.GPSTime.min()
    while True:
        if route.loc[i, 'Status'][1] == 'B':
            break
        i = i+1
    return i


def GPScallback(GPSdata):
    global flag, flag2
    global i
    global route

    print(flag)
    if flag == 0:
        i = find_initiali(route)  # TODO
        flag = 1

    # Lat_A=route.loc[i,'Lattitude']
    Lat_A, Lng_A = route.loc[i, 'Lattitude'], route.loc[i, 'Longitude']
    Lat_B, Lng_B = GPSdata.Lattitude, GPSdata.Longitude
    Heading = GPSdata.Heading
    route_Heading = route.loc[route.GPSTime.max(), 'Heading']
    xd, yd = calcDistance(Lat_B, Lng_B, Lat_A, Lng_A)

    #Lat_A1, Lng_A1=route.loc[i,'Lattitude'],route.loc[i,'Longitude']
    #xd1,yd1=calcDistance(Lat_A, Lng_A, Lat_A1, Lng_A1)
    # v=0.2
    # vxd=v*xd/((xd**2+yd**2)**0.5+0.000001)
    # vyd=v*xd/((xd**2+yd**2)**0.5+0.000001)
    # vx=GPSdata.Ve
    # vy=GPSdata.Vn

    if xd**2+yd**2 < 0.5:
        i = min(i+1, route.GPSTime.max())  # TODO

    if (xd**2+yd**2 < 0.01) or (flag2 == 1):
        flag2 = 1
        v, w = control_Heading(route_Heading, Heading)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        talker2(cmd)
        print('final point:'+str(i))
        print(sin(route_Heading*pi/180), cos(route_Heading*pi/180))
        print(sin(Heading*pi/180), cos(Heading*pi/180))

    else:
        v, w = control_to_point(xd, yd, Heading)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        talker2(cmd)
        print('point:'+str(i))
        print(xd, yd)
        print(sin(Heading*pi/180), cos(Heading*pi/180))


def talker2(cmd):
    pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # rate = rospy.Rate(10) # 10hz

    rospy.loginfo(cmd)
    pub2.publish(cmd)


if __name__ == '__main__':
    flag = 0
    flag2 = 0
    i = 1
    route = pd.read_csv('route.csv')
    route = route.set_index('GPSTime', drop=False)
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("GPSdata", myGPS, GPScallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
