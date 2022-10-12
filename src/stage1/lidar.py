#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8

forward_limit = rospy.get_param('~forward_limit', 1.5)
lr_limit = rospy.get_param('~lr_limit',4)

region_avg= [None]*6
def callback(msg):
    global region_avg
    
    ranges_list = list(msg.ranges)
    x=60*len(ranges_list)/360
    x_int = int(x/2)  #30 degree
    

    regions = {
	                     "front": [], "front_l": [], "back_r" : [],
	                     "back" : [], "back_l" : [], "front_r" : [],
	                 }

    regions_name = [
	                    "front", "front_l", "back_r" ,
	                     "back", "back_l", "front_r",
    ]

    
    # divide the data from scan topic into regions 
    # regions[]=  # 6 region 

    front = ranges_list[:x_int]+ranges_list[-(x_int):]  #(-30:30)
    regions["front"]=[x for x in front if x != float('inf') ]
    regions["front_r"] = [x for x in ranges_list[-(x_int*3):-(x_int)] if  x != float('inf')] #(-90:-30)
    regions["front_l"] = [x for x in ranges_list[x_int:x_int*3] if x != float('inf')] #(30:90)
    
    # sorted the distance in that we get in region 
    # to get the min distances in the array  # we will use only 20 element in this array to calculate the average 
    for  region in (regions_name[0:]):
        regions[region].sort()

    # calculate the average using first 20 element in the sorted regions
    for i, region in enumerate(regions_name[0:]):
        region_avg[i]=sum(regions[region][0:19])/20
    
    print("front :", region_avg[0])
    print("right :", region_avg[-1])
    print("left :", region_avg[1])

    """
        0-->front
        1-->front_l
        5-->front_r
    """

def lidar_motion():
    if region_avg[0] == 0:
        y= "forward"
        motion = 3

    elif region_avg[0] <= forward_limit :
        # if region_avg[5] <= lr_limit and region_avg[1] <= lr_limit :
        #     y="right"
        motion = 0
        # elif region_avg[5] <= lr_limit :
        #     y="left"
        #     motion = 2 
        # elif region_avg[1] <= lr_limit :   
        #     y="right"
        #     motion = 1
        
    elif region_avg[0] > forward_limit :
        y= "forward"
        motion = 3
    else:
        y= "stop"
        motion = 0
    print (motion)
    pub.publish(motion)
    
    
def listener():
    global pub
    rospy.init_node('lidar')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/motion', Int8 , queue_size=10)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown() :
        lidar_motion()
        rate.sleep()
    

if __name__ == '__main__':
    listener()
