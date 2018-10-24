#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter
#   * Modified 2018 Alexander Feldman, feldmanay@gmail.com

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

key_press_buffer = None
def key_pressed_cb(msg):
    print('called back')
    global key_press_buffer
    if str(msg.data) in ["0","1","2","3","4"]:
        key_press_buffer = str(msg.data)
    else:
        key_press_buffer = None

def get_key_press_buffer():
    global key_press_buffer
    ret_val = key_press_buffer
    key_press_buffer = None
    return ret_val


class SimpleKeyTeleop():
    def __init__(self):
        self._pub_cmd = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=100)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._backward_rate = rospy.get_param('~backward_rate', 0.5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

    movement_bindings = {
        "0": ( 0,  0), # stop
        "1": ( 1,  0), # forward
        "2": (-1,  0), # reverse
        "3": ( 0,  1), # left
        "4": ( 0, -1), # right
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while not rospy.is_shutdown():
            key = get_key_press_buffer()
            print('got buffer of ' + str(key))
            if key:
                acc = self.movement_bindings[key]
                if acc[0]:
                    linear = self._linear + acc[0]
                elif acc[1]:
                    angular = self._angular + acc[1]
                else: # stop
                    self._angular = 0
                    self._linear = 0
                    self._publish()

                self._last_pressed[key] = rospy.get_time()
                self._set_velocity()
                self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        print('keys ' + str(keys))
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _publish(self):

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)

app = SimpleKeyTeleop()
sub = rospy.Subscriber('/teleop_keypress', UInt8, key_pressed_cb)
rospy.init_node('key_teleop')
app.run()
