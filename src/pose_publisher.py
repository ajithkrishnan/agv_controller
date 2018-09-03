#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Tobias Mueller. All rights reserved.
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
#  * Neither the name of the association nor the names of its
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

"""
This script broadcasts a TF.
"""

import argparse
import time
import numpy
from math import sin, cos, atan2, hypot, fabs, sqrt, pi

import rospy
import actionlib
import tf
import tf2_ros

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from std_msgs.msg import Float64
from etarob_msgs.srv import *


class SetpointBroadcaster(object):


    def __init__(self, name):
        '''
        @brief 
        '''

        self.pub_pose_left = rospy.Publisher("/etarob/pose_left", PoseStamped, queue_size=1) 
        self.pub_pose_right = rospy.Publisher("/etarob/pose_right", PoseStamped, queue_size=1) 

        self.pose_left = PoseStamped()
        self.pose_right = PoseStamped()

        self.pose_left.header.frame_id = "base_link"
        self.pose_left.pose.position.x = 1
        self.pose_left.pose.position.y = 1
        self.pose_left.pose.position.z = -0.5

        self.pose_right.header.frame_id = "base_link"
        self.pose_right.pose.position.x = 1
        self.pose_right.pose.position.y = -1
        self.pose_right.pose.position.z = -0.5


        rospy.on_shutdown(self.cleanup)

        rospy.loginfo("Start setpoint broadcaster.")

    def cleanup(self):
        '''
        @brief destructor
        '''
        rospy.loginfo("Stop setpoint broadcaster.")


    def broadcast(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pose_left.header.stamp = rospy.Time.now()
            self.pub_pose_left.publish(self.pose_left)
            self.pose_right.header.stamp = rospy.Time.now()
            self.pub_pose_right.publish(self.pose_right)
            rate.sleep()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = __doc__)
    # Accept control rate. Default is 20 hz.
    parser.add_argument("--rate", help="Broadcast rate", type=float, default=20.0)
    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node("etarob_setpoint_broadcaster")

        sb = SetpointBroadcaster(rospy.get_name())
        

        # Keep looping unless the system receives shutdown signal.
        sb.broadcast()


    except rospy.ROSInterruptException:
        pass
