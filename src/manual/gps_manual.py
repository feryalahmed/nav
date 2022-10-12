#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32
from math import sin, cos, sqrt, atan2, radians, degrees
from gps_custom_msg_pkg.msg import gps_custom_msg

R = 6371
file_path = rospkg.RosPack().get_path('stage1_15')+"/src/position.txt"
longitude1 = 0
latitude1 = 0
yaw = 0

with open(file_path,'r') as file:
    waypoints=file.read().splitlines()
    points = len(waypoints)
    for i in range(len(waypoints)):
        waypoints[i] = waypoints[i].split()
       
def yaw_cb(msg):
    global yaw
    yaw = msg.data

def gps_cb(msg):
    global longitude1
    global latitude1
    longitude1 = msg.longitude
    latitude1 = msg.latitude

def gps_calculation():
    global points

    lon2 = [None]*(len(waypoints))
    lat2 = [None]*(len(waypoints))
    distance = [None]*(len(waypoints))
    azimuth_angle = [None]*(len(waypoints))
    angle = [None]*(len(waypoints))

    lat1 = radians(latitude1)
    lon1 = radians(longitude1)

    #list of points lon2[] , lat2[]
    for i in range (points):
        lon2[i] = radians(float(waypoints[i][0]))
        lat2[i] = radians(float(waypoints[i][1]))

    for i in range (points):
        dlon = lon1 - lon2[i]
        dlat = lat1- lat2[i]
    
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2[0]) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance[i] = R * c *1000  #meter

        x = cos(lat2[0])*sin(dlon)
        y = cos(lat1)*sin(lat2[0])-sin(lat1)*cos(lat2[0])*cos(dlon)
        az = atan2(x, y)
        azimuth_angle[i] = degrees(az)

        angle[i] =  azimuth_angle[i] + yaw

        if (angle[i] >180):
            angle[i] = 360 - angle[i]
        elif (angle[i] <-180):
            angle[i] = 360+ angle[i]

        if distance > 1 :
            if -5 < angle[i] < 5:
                y = "forward"
            elif angle[i] <0 :
                y = "right"
            elif angle[i] > 0:
                y = "left"
        else :
            y = "done"

        print("distance:"+str(i)+"=" ,distance[i]),
        print("angle:"+str(i)+"=" ,angle[i]),
        print("motion",y)
    print("______________________________________________________________________")    

def listener():
    rospy.init_node('gps_manual', anonymous=True)
    rospy.Subscriber("gps_msg", gps_custom_msg, gps_cb)
    rospy.Subscriber("yaw", Float32,yaw_cb)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown() :
        gps_calculation()
        rate.sleep()  

if __name__ == '__main__':
    try:
        listener()    
    except rospy.ROSInterruptException:
        pass
