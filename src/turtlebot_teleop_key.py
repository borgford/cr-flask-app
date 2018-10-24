#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

import sys, select, termios, tty


moveBindings = {
        '1':(1,0),
        '3':(0,1),
        '4':(0,-1),
        '2':(-1,0),
           }

speedBindings={}

buffer_key = None

def setKeyCb(msg):
    global buffer_key
    buffer_key = str(msg.data)


def getKey():
    global buffer_key
    if buffer_key is not None:
        key = buffer_key
        buffer_key = None
        return key
    return None

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


rospy.init_node('turtlebot_teleop')
pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
sub = rospy.Subscriber('/web/teleop', UInt8, setKeyCb)

x = 0
th = 0
target_speed = 0
target_turn = 0
control_speed = 0
control_turn = 0
while not rospy.is_shutdown():
    key = getKey()
    if key is None:
        if control_speed == 0 and control_turn == 0:
            continue
    elif key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
        count = 0
    elif key == '0':
        x = 0
        th = 0
        control_speed = 0
        control_turn = 0
    else:
        continue

    target_speed = speed * x
    target_turn = turn * th

    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.02 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.02 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn

    twist = Twist()
    twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    pub.publish(twist)


def shutdown():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

rospy.on_shutdown(shutdown)
