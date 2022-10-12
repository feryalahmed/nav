#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
motion = 0
limit = rospy.get_param('~limit', 2 )

# def stage3_cb(msg):
#     global stage3 
#     stage3 = msg.data


region_avg= [None]*3
def callback(msg):
    global region_avg
    
    ranges_list = list(msg.ranges)
    x=40*len(ranges_list)/360
    x_int = int(x/2)  #20 degree
    

    regions = {
	                     "front": [], "right": [], "left" : [],
	                 }

    regions_name = [
	                    "front", "right", "left", 
    ]

    
    # divide the data from scan topic into regions 
    # regions[]=  # 6 region 

    front = ranges_list[:x_int]+ranges_list[-(x_int):]  #(-20:20)
    regions["front"]=[x for x in front if x != float('inf') ]
    regions["right"] = [x for x in ranges_list[-(x_int*3):-(x_int)] if  x != float('inf')] #(-60:-20)
    regions["left"] = [x for x in ranges_list[x_int:x_int*3] if x != float('inf')] #(20:60)
    
    # sorted the distance in that we get in region 
    # to get the min distances in the array  # we will use only 20 element in this array to calculate the average 
    for  region in (regions_name[0:]):
        regions[region].sort()

    # calculate the average using first 20 element in the sorted regions
    for i, region in enumerate(regions_name[0:]):
        region_avg[i]=sum(regions[region][0:19])/20
        
    print("front :", region_avg[0])
    print("right :", region_avg[1])
    print("left :", region_avg[2])

    min_dist = min(region_avg)
    min_index = region_avg.index(min_dist)
    print(min_index)
    print("arr",region_avg)
    """
        0-->front
        1-->right
        -1-->left
    """


    if(min_dist >= limit):
        
        if(min_index == 0):
            #move forward
            motion = 3
        
        elif(min_index == 1):
            # move right
            motion = 1
            
        elif(min_index == 2):
            #move left 
            motion = 2
        
    else:
        # stop
        motion = 0

    pub.publish(motion)



        
def listener():
    global pub
    rospy.init_node('stage3_v2')
    rospy.Subscriber('/scan', LaserScan, callback)
    # rospy.Subscriber('/stage3', Int8, stage3_cb)
    
    pub = rospy.Publisher('/motion', Int8 , queue_size=10)
    rate=rospy.Rate(10)
    rospy.spin()
    

if __name__ == '__main__':
    listener()
