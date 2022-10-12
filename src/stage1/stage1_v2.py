#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32,Int8,Int16
from math import pi
ticks_flag = False
counter_flag = True
prev_lencoder = 0
lidar_motion =0
lmult=0
ticks1=0
ticks_old =0
encoder_min = -32768
encoder_max = 32768
encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min
encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min
stage = 0


def stage_cb(msg):
    global stage
    stage = msg.data

def ticks1_cb(msg):
    global ticks1
    global lmult
    global prev_lencoder
    global left
    ticks1 = msg.data
    enc = ticks1
    
    if (enc < encoder_low_wrap and prev_lencoder > encoder_high_wrap):
        lmult = lmult + 1
        
    if (enc > encoder_high_wrap and prev_lencoder < encoder_low_wrap):
        lmult = lmult - 1
        
    left = 1.0 * (enc + lmult * (encoder_max - encoder_min)) 
    prev_lencoder = enc

def lidar_cb(msg):
    global lidar_motion
    lidar_motion = msg.data

def gps_imu_cb(msg):
    global gps_imu_motion 
    gps_imu_motion = msg.data

    motion()

def motion():
    global ticks_flag
    global counter_flag
    global left
    global ticks_old

    if stage == 0 :

        # gps&imu = 3       lidar = 3 
        # best case
        if gps_imu_motion == 3 and lidar_motion == 3 :
            pub.publish(gps_imu_motion)
            ticks_flag = False

        # gps&imu = 3       lidar = 2||1
        # there is a box between the robot and the waypoint 
        elif gps_imu_motion == 3 and (lidar_motion == 2 or lidar_motion == 1 ):
            pub.publish(lidar_motion)
            ticks_flag = False

        # gps&imu = 1||2        lidar = 3 
        # in 2 cases 
        # 1- robot already rotated to avoid the box ,but still in the same point // ticks_flag = false
        # 2- robot moved to the save point and want to rotate to reach the waypoint // ticks_flag = true
        elif (gps_imu_motion == 1 or gps_imu_motion == 2) and lidar_motion == 3 :
            if ticks_flag == False:
                pub.publish(lidar_motion)

                if counter_flag == True :
                    ticks_old = left
                    counter_flag = False

                if left >= ticks_old +(600/(2*pi*0.13)*2):
                    ticks_flag = True
                    counter_flag = True

            elif ticks_flag == True :
                pub.publish(gps_imu_motion)
                

        # gps&imu = 1||2        lidar = 1||2 
        elif (gps_imu_motion == 1 or gps_imu_motion == 2) and (lidar_motion == 2 or lidar_motion == 1 ) :
            pub.publish(lidar_motion)

        # gps&imu = 0       lidar = 1||2||3
        # the robot reached its goal
        elif gps_imu_motion == 0 and (lidar_motion == 1 or lidar_motion == 2 or lidar_motion == 3) :
            pub.publish(gps_imu_motion)

        # gps&imu = 1||2||3      lidar = 0
        # emergency case
        else : 
            pub.publish(lidar_motion)
    else:
        pub.publish(0)

def listener():
    global pub
    rospy.init_node('stage1', anonymous=True)

    rospy.Subscriber("stage", Int8 ,stage_cb)
    rospy.Subscriber("left_ticks", Int16,ticks1_cb )
    rospy.Subscriber("lidar_motion",Int8 , lidar_cb)
    rospy.Subscriber("gps_motion", Int8 ,gps_imu_cb)
    
    
    pub = rospy.Publisher('motion', Int8, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    listener()    
