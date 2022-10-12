#!/usr/bin/env python
import rospkg
from math import atan, cos, degrees, hypot, radians, sin, atan2 ,pi
import rospy
from std_msgs.msg import *

ticksperMeter = 1200/(2 * pi * 0.125)
encoder_min = -32768
encoder_max = 32768
encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min
encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min
tick1 = 0
lmult = 0
prev_lencoder = 0
left = 0
move = 0
resetflag = 0
accumulated_dist = 0
current_dist = 0
yaw = 1000
obstacle_detected = 0
yaw_recorded = 0
stage3 = 0

# stationary = 0
# rotate = 1
# moving = 2
status = 1
index = 1 # Index starts as 1 since the lists of x_cor and y_cor already has 0 

NoOfPoints = rospy.get_param('~NoOfPoints',3)
file_path = rospkg.RosPack().get_path('nav')+"/src/stage2/points.txt"
    

def yaw_cb(msg):
    global yaw
    yaw = msg.data

def ticks1_cb(msg):
    global ticks1
    global lmult
    global prev_lencoder
    global left
    global current_dist
    global status
    global current_dist

    ticks1 = msg.data
    enc = ticks1

    if (enc < encoder_low_wrap and prev_lencoder > encoder_high_wrap):
        lmult = lmult + 1
        
    if (enc > encoder_high_wrap and prev_lencoder < encoder_low_wrap):
        lmult = lmult - 1
    left = 1.0 * (enc + lmult * (encoder_max - encoder_min)) # Handling overflow
    prev_lencoder = enc

    current_dist = float(left) / ticksperMeter

def rotate(target_angle):
    global status
    global resetflag
    resetflag = 0
    direction = 0
    if target_angle >= 0 and target_angle <= yaw <= target_angle + 180:
        direction = -1
    elif target_angle <=0 and target_angle - 180 <= yaw <= target_angle:
        direction = 1

    print('My Status =' + str(status))
    print('My Angle = ' + str(yaw)),
    #print('difference angle =' + str(target_angle - yaw))
    print('Current Destination angle = ' + str(target_angle))
    
    if ((not(target_angle - 5 < yaw < target_angle + 5)) and (status == 1)):
        # Error of 5 de  grees
        # if the yaw angle is not between target angle + 5 or not between target angle - 5
        # then keep rotating
        # else stop
        print('Rotating')
        # print(diff_ang)
        if(direction >= 0):
            # Rotate Right
            move = 1
        elif(direction < 0):
            #mo Rotate left
            move = 2
        else:
            move = 0
        print(move)

    
    else:
        # move
        status = 2
        move = 0
    pub.publish(move)
    rate.sleep()
        

def moving(target_dist):
    global resetflag
    global current_dist
    global status
    global yaw
    global index
    if resetflag == 0:
        resetflag = 1
        global accumulated_dist
        accumulated_dist = current_dist
    
    current_dist = current_dist - accumulated_dist

    print('My Status = ' + str(status))
    print('My distance from last point = ' + str(current_dist)),
    print('My Target Distance = ' + str(target_dist))

    if ((not(target_dist - 0.25 < current_dist < target_dist + 0.25)) and (status == 2)): 
        #Error of 0.25 meters
        # if the current distance of the rover is bigger that target distance - 5
        # and less than target distance + 0.5
        # then move forward
        # else stop
        print('Moving')
        # Forward
        move = 3 
    else:
        # Rotate Again
        status = 1
        index += 1
        move = 0
        yaw_recorded = 0
    pub.publish(move)
    rate.sleep()

def get_targetAngle(x1, y1 , x2 ,y2):
    #North with Y_axis
    #print(x1,y1,x2,y2)
    if x2 >= x1 and y2 >= y1: # First Quad
        return degrees(atan((x2 - x1)/(y2 - y1)))
    elif x2 >= x1 and y2 <= y1: # 4th Quad
        return 180 + degrees(atan((x2 - x1)/(y2 - y1)))
    elif x2 <= x1 and y2 >= y1: #2rd Quad
        return degrees(atan((x2 - x1)/(y2 - y1)))
    elif x2 <= x1 and y2 <= y1: #3rd Quad
        return degrees(atan((x2 -x1)/(y2 - y1))) - 180

    # tan^-1(x2 - x1)/(y2 - y1)
    # -30 = -X + 90

def get_targetDist(x1, y1, x2, y2):
    return hypot(x2-x1,y2-y1)
    # Return euclidean distance sqrt(x*x + y*y)

def listener():
    global pub
    global rate
    global yaw_recorded
    global status
    global stage3
    x_coordinates = [0]
    y_coordinates = [0]
    
    with open(file_path,'r') as file:
        waypoints=file.read().splitlines()
        points = len(waypoints)
        for i in range(len(waypoints)):
            waypoints[i] = waypoints[i].split()
            
            x_coordinates.append(float(waypoints[i][0]) * sin(radians(float((waypoints[i][1])))))
            y_coordinates.append(float(waypoints[i][0]) * cos(radians(float((waypoints[i][1])))))
    rospy.init_node('stage2', anonymous=True)
    rospy.Subscriber('yaw', Float32, yaw_cb)
    rospy.Subscriber('left_ticks', Int16, ticks1_cb)
    pub = rospy.Publisher('motion', Int8, queue_size=1)
    global index
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("My Current Point" + str(index))
        if (index == len(x_coordinates)):
            print("Stopping")
            if stage3 == 0:
                pub.publish(0)
                stage3 = 1
        else:
            print(x_coordinates)
            print(y_coordinates)
            if status == 1:
                target_angle = get_targetAngle(x_coordinates[index-1],y_coordinates[index - 1],
                x_coordinates[index], y_coordinates[index])
                rotate(target_angle)
            if status == 2:
                target_dist = get_targetDist(x_coordinates[index-1], y_coordinates[index-1],
                x_coordinates[index], y_coordinates[index])
                moving(target_dist)
            

    rospy.spin()

if __name__ == '__main__':
    global stage3
    if (stage3 != 1):
        listener()  