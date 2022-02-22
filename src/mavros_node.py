#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics

import rospy
import thread
import threading
import time
import mavros

#import low_pass

from numpy import linalg
import numpy as np

from math import *
from mavros.utils import *
from mavros.param import *
from mavros import setpoint as SP
from std_msgs.msg import Header
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import PositionTarget, RCIn
from sensor_msgs.msg import NavSatFix, Range, LaserScan
#from gazebo_msgs.msg import ModelStates

class VisionPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.errorDx = 0.0
        self.errorDy = 0.0
        self.errorDz = 0.0
        self.armed = False
        self.pitch = 0.0
        self.roll = 0.0
        self.timeout = 180
        self.count = 0
        self.error_updated = [False, False]
        self.descent = False
        self.z = 0.0
        self.fakeX = 0.0
        self.fakeY = 0
        self.spX = 0.0
        self.spY = 0
        self.rcX = 0
        self.rcY = 0

        self.rcX_trim = 1489
        self.x_max = 1.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcX_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingX = self.x_max/self.rcX_max # scaling factor for rcIn to posX

        self.rcY_trim = 1490
        self.y_max = 100.0/100.0 #max x position for UAV to chase. convert from 5cm to metres. x_max is in metres
        self.rcY_max = 500.0 #max range of rc from centre trim of 1500
        self.scalingY = self.y_max/self.rcY_max # scaling factor for rcIn to posX

        self.freq = 20.0
        # self.lpDx = low_pass.lowpassfilter(1.0/self.freq, 0.01)
        # self.lpDz = low_pass.lowpassfilter(1.0/self.freq, 0.01)
        # self.lpDy = low_pass.lowpassfilter(1.0/self.freq, 0.01)
        #self.lpSx = low_pass.lowpassfilter(1.0/self.freq, 3.0)

        rospy.Subscriber("/pose_correction", Point, self.position_callback)
        # rospy.Subscriber("mavros/setpoint_raw/target_local", PositionTarget, self.setpoint_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        rospy.Subscriber("mavros/distance_sensor/hrlv_ez4_pub", Range, self.error_lpZ, queue_size=1)
        #rospy.Subscriber("mavros/rc/in", RCIn, self.updateRCIn, queue_size=1)
        #rospy.Subscriber("teraranger0/laser/scan", LaserScan, self.range_callback)
        rospy.Subscriber("error_dx", Float32, self.error_dx)
        #rospy.Subscriber("gazebo/model_states", ModelStates, self.gazebo_pose)
        rospy.Subscriber("error_dy", Float32, self.error_dy)
        rospy.Subscriber("error_dz", Float32, self.error_dz)
        rospy.Subscriber("roll", Float32, self.error_roll)
        rospy.Subscriber("pitch", Float32, self.error_pitch)

        self.pub_lpe = rospy.Publisher('VISION_POSITION_ESTIMATE', PoseStamped, queue_size=1)
        self.rate = rospy.Rate(self.freq) # 20hz
        self.has_global_pos = True
        self.local_position = PoseStamped()
        self.setpointX = 0


        # self.filterDx = 0
        # self.filterDy = 0

        while not rospy.is_shutdown():
            # print 'running'

            self.rate.sleep()
            self.lpe(self.errorDx)
            # self.lpe(self.errorDx) #for using lsq errorDx = 0
            # self.lpe(self.fakeX) #for using fakeX toggled by RCIn[7]

    #
    # General callback functions used in tests
    #

    def lpe(self, errorDx):

        # forward left up: = [ self.errorDx, self.errorDy, self.z ]
        # east north up: [-self.errorDy, self.errorDx, self.z]

        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "local_origin"

        ## get external sensor position correction data
        pos.pose.position = self.local_position

        # base link enu i.e. flu
        # we need to substract pi/2 from the number, as 0 is east. we want tunnel axis as north
        q = quaternion_from_euler(self.roll, self.pitch, self.errorDz + np.pi/2)

        # these needs to be in enu base_link
        pos.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # update timestamp for each published SP
        pos.header.stamp = rospy.Time.now()
        self.pub_lpe.publish(pos)

    def position_callback(self, data):
        self.local_position = data

    def setpoint_callback(self, data):
        self.spX = data.position.y #because of ned to enu shit
        self.spY = -data.position.x

    def global_position_callback(self, data):
        self.has_global_pos = True

    def range_callback(self, msg):
        self.z = msg.ranges[0]
        if self.z >= 5:
            self.z = 0

    def error_roll(self, msg):
        self.roll = msg.data

    def error_pitch(self, msg):
        self.pitch = msg.data

    def gazebo_pose(self, msg):
        # print msg.pose[2].position.x
        self.errorDx = msg.pose[2].position.x
        self.error_updated[0] = True

    def error_dx(self, msg):
        self.errorDx = msg.data
        # self.errorDx = self.lpDx.update_filter(msg.data)
        self.error_updated[0] = True

    def error_dy(self, msg):
        self.errorDy = msg.data
        # self.errorDy = self.lpDy.update_filter(msg.data)
        self.error_updated[1] = True

    def error_dz(self, msg):
        self.errorDz = msg.data
        # self.errorDz = self.lpDz.update_filter(msg.data)

    def error_lpZ(self, msg):
        self.z = msg.range

    def updateRCIn(self, msg):
        self.rcX = msg.channels[5] - self.rcX_trim
        spX = self.rcX * self.scalingX  # [m] = [pwm] x [m/pwm]
        spX = -spX
        self.spX = self.lpSx.update_filter(spX)
        #absolute difference in x from the setpointX
        # self.fakeX = self.spX - tempX # +ve tempX implies
        #print self.spX

        # self.rcY = msg.channels[5] - self.rcY_trim
        # self.fakeY = self.rcY * self.scalingY #absolute difference in x from the setpointX
        #print self.spX


if __name__ == '__main__':
    rospy.init_node('vision_test_node')

    node = VisionPosition()

    rospy.spin()