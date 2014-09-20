#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

"""I know globals are bad, but it's late and I want to go to bed ): 
Ideally, these would be made into classes. Maybe next time..."""
nose_distance_to_wall = 1.1
side_distance_to_wall = 1.1
parallel_to_wall = False
turn = 0

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global parallel_to_wall
    global nose_distance_to_wall
    global side_distance_to_wall
    global turn
    
    """Here we define first loop, where we search for a wall"""
    if not parallel_to_wall:
        valid_measurements = []
        for i in range(5):
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                valid_measurements.append(msg.ranges[i])
        if len(valid_measurements):
            nose_distance_to_wall = sum(valid_measurements)/float(len(valid_measurements))
            turn = 0
        else:
            turn = -0.1
        print "Nose", nose_distance_to_wall
        
    elif parallel_to_wall:
        valid_measurements = []
        for i in range(85, 95):
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                valid_measurements.append(msg.ranges[i])
        if len(valid_measurements):
            side_distance_to_wall = sum(valid_measurements)/float(len(valid_measurements))
            turn = 0
        else:
            side_distance_to_wall = 1.1
            turn = 0.1
        if side_distance_to_wall > 2 or side_distance_to_wall < 0.5:
            parallel_to_wall = False
        print "Side", side_distance_to_wall
    
   
def wall():
    """ Run loop for the wall node """
    global side_distance_to_wall
    global nose_distance_to_wall
    global parallel_to_wall
    global turn
    

    while not rospy.is_shutdown():
        if 0.95 < nose_distance_to_wall < 1.05 and not parallel_to_wall:
            msg = Twist() #we don't send any messages if we're close to the wall
            parallel_to_wall = True
        elif not parallel_to_wall:
            msg = Twist(linear=Vector3(x=(nose_distance_to_wall-1)*.2))
            msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = turn
        elif parallel_to_wall:
            msg = Twist()
            msg.linear.x = 0.1; msg.linear.y = 0; msg.linear.z = 0
            msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = turn
        pub.publish(msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('/scan', LaserScan, scan_received)
        rospy.init_node('wall', anonymous=True)
        r = rospy.Rate(10) # 10hz
        wall()
    except rospy.ROSInterruptException: pass
