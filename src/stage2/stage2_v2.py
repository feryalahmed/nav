#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int8
from nav_msgs.msg import Odometry
from math import atan

# [ [x,y] ]
target = [ [3,2] ]
points = len(target)
# print(points)
reached = 0
x_reached = False
y_reached = False

yaw = 0
motion = 0

odom_x = 0
odom_y = 0

target_angle = 0
# angle_threshold = 2.5
# distance_threshold = 1
stage = 2

def odom_cb(msg):
    global odom_x
    global odom_y
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

def stage_cb(msg):
    global stage
    if(msg.data==3):
        stage = msg.data

def yaw_cb(msg):
    global yaw
    # print(msg.data)\
    yaw = (-1)*msg.data
    while(yaw>360):
        yaw = yaw - 360
    while(yaw<0):
        yaw = yaw + 360
    print(yaw)
    # print("--------------")


def true_heading(x_target, y_target):
    dy = y_target-odom_y
    dx = x_target-odom_x

    # if dy==0 and dx==0:
    #     desired_angle = 0
    if dy==0:
        if x_target>=odom_x:
            desired_angle = 0
        elif x_target<odom_x:
            desired_angle = 180
    elif dx==0:
        if y_target>=odom_y:
            desired_angle = 90
        elif y_target<odom_y:
            desired_angle = 270
    
    elif dy>0 and dx>0: # 1st
        desired_angle = atan(dy/dx)
    elif dy>0 and dx<0: # 2
        desired_angle = 180 + atan(dy/dx)
    elif dy<0 and dx>0: # 4
        desired_angle = atan(dy/dx)
    elif dy<0 and dx<0: # 3
        desired_angle = 180 + atan(dy/dx)
    
    while(desired_angle<0):
        desired_angle = desired_angle + 360
    while(desired_angle>360):
        desired_angle = desired_angle - 360

    return desired_angle


def motion_fn():
    global motion
    global target_angle
    global reached
    global x_reached
    global y_reached
    global stage

    if reached==points:
        stage = 4
        pub.publish(0)
        pub_stage.publish(stage)

    if stage==3:

        if reached<points:
            target_angle = true_heading(target[reached][0], target[reached][1])
        if target[reached][0]-distance_threshold < odom_x < target[reached][0]+distance_threshold:
            x_reached = True
        else:
            x_reached = False
        if target[reached][1]-distance_threshold < odom_y < target[reached][1]+distance_threshold:
            y_reached = True
        else:
            y_reached = False

        if x_reached and y_reached:
            reached = reached+1
        else:
            pass

        # print(angle_threshold)
        # print(yaw)
        # print(target_angle)
        # print("-----------")
        # print(target_angle-yaw)
        # print("-----------")
        diff = target_angle-yaw
        if(diff>180):
            diff = 360 - diff
        elif(diff<-180):
            diff = 360 + diff

        if -angle_threshold < diff < angle_threshold:
            motion = 3 # forward
        elif diff<0:
            motion = 1 # right
        elif diff>=0:
            motion = 2 # left
        else:
            motion = 0 # stop
    
        pub.publish(motion)


def listener():
    global pub
    global pub_stage
    global stage
    global distance_threshold
    global angle_threshold
    rospy.init_node('motion', anonymous=True)
    Rate = rospy.get_param('~rate',10)
    # stage = rospy.get_param('~stage',0)
    distance_threshold = rospy.get_param('~distance_threshold',0.15)
    angle_threshold = rospy.get_param('~angle_threshold',10)
    rospy.Subscriber('yaw', Float32, yaw_cb)
    rospy.Subscriber('odom', Odometry, odom_cb)
    rospy.Subscriber('stage2', Int8, stage_cb)
    pub = rospy.Publisher('motion', Int8, queue_size=1)
    pub_stage = rospy.Publisher('stage3', Int8, queue_size=1)
    rate = rospy.Rate(Rate)
    while not rospy.is_shutdown():
        motion_fn()
        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

