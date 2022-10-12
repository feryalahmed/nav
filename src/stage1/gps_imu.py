#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Int8
wait_flag = False
distance= 0
azimuth_angle=0
yaw=0

yaw_out=0
azimuth_out=0
distance_out=0
stage = 0

limit_out=10
def points_cb(msg):
    global wait_flag
    wait_flag = True

def stage_cb(msg):
    global stage
    stage = msg.data

def distance_cb(msg):
    global distance
    global distance_out
    distance = msg.data
    distance_out=0

def azimuth_angle_cb (msg):
    global azimuth_angle
    global azimuth_out
    azimuth_angle = msg.data
    azimuth_out =0

def yaw_cb (msg):
    global yaw
    global yaw_out
    yaw = msg.data
    yaw_out =0
    # rospy.logwarn("yaw")

def motion():

    global move
    angle =  azimuth_angle + yaw
    # angle correction
    if (angle >=180):
        angle = 360 - angle
    elif (angle <-180):
        angle = 360+ angle

    pub_angle.publish(angle)
    # idea: detect decrement
    if distance > limit_dis:
        
        if -1*limit_angle < angle < limit_angle:
            y = "forward"
            move = 3
        elif angle <0 :
            y = "right"
            move = 1
        elif angle > 0:
            y = "left"
            move = 2
    else :
        move =0


           
def listener():
    global pub
    global pub_angle
    global rate

    global limit_angle
    global limit_dis

    global azimuth_out
    global distance_out
    global yaw_out

    global move
    global wait_flag
    safe_flag_imu = True
    safe_flag_gps = True
    
    rospy.init_node('motion', anonymous=True)

    limit_dis = rospy.get_param('~limit_dis',2)  
    limit_angle = rospy.get_param('~limit_angle',5)  
    rate = rospy.get_param('~rate',10) 
    wait = rospy.get_param('~wait',4) 

    rospy.Subscriber("distance",Float32 , distance_cb)
    rospy.Subscriber("azimuth_angle",Float32 , azimuth_angle_cb)
    rospy.Subscriber("yaw", Float32,yaw_cb)
    rospy.Subscriber("points",Int8, points_cb)
    rospy.Subscriber("stage1",Int8, stage_cb)
    
    pub = rospy.Publisher('gps_motion', Int8, queue_size=limit_out) # motion 
    pub_angle = rospy.Publisher('angle', Float32, queue_size=10)

    rate_publish =rospy.Rate(rate)
    rate_sleep = rospy.Rate(1)

    while not rospy.is_shutdown() :
        if stage == 0 :
            if yaw_out >=limit_out and safe_flag_imu:
                rospy.logwarn("imu stopped ")
                safe_flag_imu=False

            if azimuth_out >=limit_out and safe_flag_gps:
                rospy.logwarn("gps stopped ")
                safe_flag_gps = False

            if yaw_out<limit_out and azimuth_out <limit_out :

                if not safe_flag_gps :
                    rospy.logwarn("gps connected")
                if not safe_flag_imu :
                    rospy.logwarn("imu connected")

                motion()

                if move:
                    pub.publish(move)
                    rate_publish.sleep()
                elif (wait_flag):
                    print("i want to stop , help me")
                    wait_flag = False
                    for i in range(wait):
                        print(i) 
                        pub.publish(move)
                        rate_sleep.sleep()
                    print("i will move to next point , friend")
                    
            else:
                pub.publish(0)
                rate_publish.sleep()
            
            azimuth_out+=1
            distance_out+=1
            yaw_out+=1


if __name__ == '__main__':
    try:
        listener()    
    except rospy.ROSInterruptException:
        pass
