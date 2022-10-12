#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8,Int16,Float32

stage = 1
yaw=700
rticks = 0
lticks = 0
motion = 0

def stage_cb(msg):
    global stage
    stage = msg.data

def yaw_cb(msg):
    global yaw
    yaw = msg.data

def reset_fn():
    global stage
    global motion
    
    if stage == 1 :
        if not(-5<yaw<5 ):
            if yaw > 0 :
                motion = 2 #left
            elif yaw < 0 :
                motion = 1 #right
            pub_motion.publish(motion)
            # rate.sleep()
        else:
            stage=0
            print("stage 2 starts")
            for i in range(5):  
                motion = 0
                pub_motion.publish(motion)
                rospy.sleep(1)
                pub_stage.publish(stage)
    

    

def listener():
    global pub_stage
    global pub_motion
    global rate

    rospy.init_node('reset', anonymous=True)
    rospy.Subscriber("stage1", Int8, stage_cb)
    rospy.Subscriber("yaw", Float32,yaw_cb )

    pub_stage = rospy.Publisher("stage2", Int8, queue_size=10)
    pub_motion = rospy.Publisher("motion", Int8, queue_size=10)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown() :
        reset_fn()
        rate.sleep()
        


if __name__ == '__main__':
    try:
        listener() 
    except rospy.ROSInterruptException:
        pass