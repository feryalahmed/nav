#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import Float32,Int8
from math import sin, cos, sqrt, atan2, radians, degrees
from gps_custom_msg_pkg.msg import gps_custom_msg

gps_flag = False
R = 6371
start=0
stage =0
file_path = rospkg.RosPack().get_path('nav')+"/src/stage1/position.txt"
#read from file
with open(file_path,'r') as file:

    waypoints=file.read().splitlines()
    points = len(waypoints)
    for i in range(len(waypoints)):
        waypoints[i] = waypoints[i].split()

lon2 = radians(float(waypoints[0][0]))
lat2 = radians(float(waypoints[0][1]))

def gps(msg):
    global gps_flag
    global longitude1
    global latitude1
    longitude1 = msg.longitude
    latitude1 = msg.latitude
    gps_flag = True

def gps_calculation():
    global lon2 
    global lat2
    global start
    global points
    global finish
    
    lat1 = radians(latitude1)
    lon1 = radians(longitude1)

    dlon = lon1 - lon2
    dlat = lat1- lat2 

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c *1000  #meter

    pub_distance.publish(distance)

    x = cos(lat2)*sin(dlon)
    y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon)
    angle = atan2(x, y)
    angle = degrees(angle)
    pub_angle.publish(angle)

    # move to next waypoint
    # print(points,start)
    
    if distance < limit_dis and (points-1)>=0 :
        if(points-1):
            lon2 = radians(float(waypoints[start+1][0]))
            lat2 = radians(float(waypoints[start+1][1]))
            start = start + 1
        
        points-=1
        print(points)
        pub_points.publish(points) 
        if(points == 0):
            stage=1
            pub_stage.publish(stage) 
       
    
def listener():
    global gps_flag
    global pub_distance
    global pub_angle
    global limit_dis
    global pub_points
    global pub_stage
    global rate

    rospy.init_node('gps', anonymous=True)
    limit_dis = rospy.get_param('~limit_dis',2)  
    rate_publish = rospy.get_param('~rate',10)

    rospy.Subscriber("gps_msg", gps_custom_msg, gps)
    pub_distance = rospy.Publisher('distance', Float32, queue_size=10)
    pub_angle = rospy.Publisher('azimuth_angle', Float32, queue_size=10)
    pub_points = rospy.Publisher('points', Int8, queue_size=10) # if point changes , wait  
    pub_stage = rospy.Publisher('stage1', Int8, queue_size=10) # stop in stage1 code
    rate=rospy.Rate(rate_publish)

    while not rospy.is_shutdown() :
        if (gps_flag):
            gps_calculation()
            gps_flag = False
        rate.sleep()  


if __name__ == '__main__':
    try:
        listener()    
    except rospy.ROSInterruptException:
        pass


