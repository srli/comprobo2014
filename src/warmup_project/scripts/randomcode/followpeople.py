#!/usr/bin/env python
"""
Created on Fri Sep 19 01:11:31 2014

@author: sophie
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

import math

distance_to_person = 1.1
turn = 0
prev_closest = 0

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global distance_to_person
    global turn
    global prev_closest
    global box_angle
    closest_indices = []
    closest_distances = []
    for i in range(-8, 8):
        if msg.ranges[i] > 0.1 and msg.ranges[i] < 5:
            closest_indices.append(i)
            closest_distances.append(msg.ranges[i])
    if len(closest_indices) > 1:
        closest_degree = sum(closest_indices)/float(len(closest_indices))
        distance_to_person = sum(closest_distances)/float(len(closest_distances))
        turn =  closest_degree
        if abs(turn) > 10:
            turn = 2
        prev_closest = closest_degree
        print "Degree", closest_degree
        print "Distance", distance_to_person
    else:
        turn = 0
        distance_to_person = 1.1
        

def wall():
    global distance_to_person
    global turn
    
    while not rospy.is_shutdown():
        if 0.95 < distance_to_person < 1:
            msg = Twist() #we don't send any messages if we're close
        else:
            msg = Twist(linear=Vector3(x=(distance_to_person - 1)*.7))
            msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = turn*0.2
        pub.publish(msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        """ Run loop for the wall node """
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('/scan', LaserScan, scan_received)
        rospy.init_node('wall', anonymous=True)
        r = rospy.Rate(10) # 10hz
    
        box_angle = int(math.tan(0.5/0.2))
    
        wall()
    except rospy.ROSInterruptException: pass