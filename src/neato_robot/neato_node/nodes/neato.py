#!/usr/bin/env python

# ROS node for the Neato Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for Neato XV-11 Robot Vacuum.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import time
import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from neato_driver.neato_driver import xv11, BASE_WIDTH, MAX_SPEED

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        self.tf_prefix = rospy.get_param('~tf_prefix', "")
        self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        rospy.loginfo("Using port: %s"%(self.port))

        self.robot = xv11(self.port)

        rospy.Subscriber("pi_cmd",String,self.pi_command)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('scan', LaserScan)
        self.odomPub = rospy.Publisher('odom',Odometry)
        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_to_send = None

        self.cmd_vel = [0,0]

    def pi_command(self,msg):
        self.cmd_to_send = msg

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = 0
        scan.angle_max = 6.26
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
    
        # main loop of driver
        r = rospy.Rate(5)
        rospy.sleep(4)
        self.robot.requestScan()
        scan.header.stamp = rospy.Time.now()
        last_motor_time = rospy.Time.now()
        total_dth = 0.0
        while not rospy.is_shutdown():
            if self.cmd_to_send != None:
                self.robot.send_command(self.cmd_to_send)
                self.cmd_to_send = None

            t_start = time.time()
            (scan.ranges, scan.intensities) = self.robot.getScanRanges()
            print 'Got scan ranges %f' % (time.time() - t_start)

            # get motor encoder values
            curr_motor_time = rospy.Time.now()
            t_start = time.time()
            try:
                left, right = self.robot.getMotors()
                # now update position information
                dt = (curr_motor_time - last_motor_time).to_sec()
                last_motor_time = curr_motor_time

                d_left = (left - encoders[0])/1000.0
                d_right = (right - encoders[1])/1000.0

                encoders = [left, right]
                dx = (d_left+d_right)/2
                dth = (d_right-d_left)/(BASE_WIDTH/1000.0)
                total_dth += dth

                x = cos(dth)*dx
                y = -sin(dth)*dx

                self.x += cos(self.th)*x - sin(self.th)*y
                self.y += sin(self.th)*x + cos(self.th)*y
                self.th += dth

                # prepare tf from base_link to odom
                quaternion = Quaternion()
                quaternion.z = sin(self.th/2.0)
                quaternion.w = cos(self.th/2.0)

                # prepare odometry
                odom.header.stamp = curr_motor_time
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quaternion
                odom.twist.twist.linear.x = dx/dt
                odom.twist.twist.angular.z = dth/dt
                self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), curr_motor_time, self.tf_prefix + "/base_link", self.tf_prefix + "/odom" )
                self.odomPub.publish(odom)
                print 'Got motors %f' % (time.time() - t_start)
            except:
                pass
            t_start = time.time()            
            # send updated movement commands
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
            print 'Set motors %f' % (time.time() - t_start)


            # publish everything
            self.scanPub.publish(scan)
            self.robot.requestScan()
            scan.header.stamp = rospy.Time.now()

            # wait, then do it again
            r.sleep()

    def cmdVelCb(self,req):
        x = req.linear.x * 1000
        th = req.angular.z * (BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
        self.cmd_vel = [ int(x-th) , int(x+th) ]
        print self.cmd_vel, "SENDING THIS VEL"

if __name__ == "__main__":
    robot = NeatoNode()
    robot.spin()

