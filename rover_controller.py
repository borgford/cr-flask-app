#! /usr/bin/env python

import rospy
import json
import pygame
import actionlib
import random
from std_msgs.msg import UInt8, String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Pose
from roving_ben_and_alex.msg import RoverStatusMessage, NavigationControllerAction, NavigationControllerGoal

ROBOT_STATES_FILE = "/home/alexander-feldman/catkin_ws/src/Gen-2-Starter-Code/files/robot_states.json"

with open(ROBOT_STATES_FILE,'r') as f:
    robot_states = json.load(f)

def vitals_cb(msg):
    if msg.level >= 5:
        print("Critical system error")
        print(state_code['name'],state_code['description'])
        print(msg.details)
        robot_state_pub.publish(10)

        print("FREAKING_OUT")
        emergency_twist_pub = rospy.Publisher('/cmd_vel_mux/input/switch', Twist, queue_size=1)
        freak_out = Twist()
        freak_out.angular_z = 2.84
        emergency_twist_pub.publish(freak_out)
        rospy.sleep(2)

        rospy.signal_shutdown()
    elif msg.level >= 3:
        pass # TODO
    else:
        pass

def nav_cb(feedback):
    print("Elapsed time: {}".format(feedback.elapsed_time.to_sec()))
    print("Estimated time remaining: {}".format(feedback.estimated_time_remaining.to_sec()))
    print("Distance traveled: {}".format(feedback.distance_traveled))
    print("Time since fiducial: {}".format(feedback.time_since_fiducial.to_sec()))
    print("Distance since fiducial: {}".format(feedback.distance_since_fiducial))
    print("")

    if feedback.estimated_time_remaining.to_sec() > 15:
        if not pygame.mixer.music.get_busy():
            print("playing music while we wait")
            pygame.mixer.music.load('/home/alexander-feldman/catkin_ws/src/Gen-2-Starter-Code/files/glass-bass-music-track.mp3')
            pygame.mixer.music.play()

    elif feedback.estimated_time_remaining.to_sec() < 5:
        if pygame.mixer.music.get_busy():
            print("this is no time for music!")
            pygame.mixer.music.stop()

last_im_pub = None
def im_cb(msg):
    global last_im_pub
    if last_im_pub is not None:
        if time.time() - last_im_pub  < .1:
            return
        else:
            last_im_pub = time.time()
            web_camera_pub.publish(msg)

status = 0
target_linear_vel = 0
target_angular_vel = 0
control_linear_vel = 0
control_angular_vel = 0

def teleop_cb(msg):
    print ("message in")
    global status
    global target_linear_vel
    global target_angular_vel
    global control_linear_vel
    global control_angular_vel

    if msg.data == 1:
        target_linear_vel = target_linear_vel + 0.01
        status = status + 1
    elif msg.data == 2:
        target_linear_vel = target_linear_vel - 0.01
        status = status + 1
    elif msg.data == 3:
        target_angular_vel = target_angular_vel + 0.1
        status = status + 1
    elif msg.data == 4:
        target_angular_vel = target_angular_vel - 0.1
        status = status + 1
    elif msg.data == 0:
        target_linear_vel   = 0
        control_linear_vel  = 0
        target_angular_vel  = 0
        control_angular_vel = 0

    if target_linear_vel > control_linear_vel:
                control_linear_vel = min( target_linear_vel, control_linear_vel + (0.01/4.0) )
    else:
        control_linear_vel = target_linear_vel

    if target_angular_vel > control_angular_vel:
        control_angular_vel = min( target_angular_vel, control_angular_vel + (0.1/4.0) )
    else:
        control_angular_vel = target_angular_vel

    twist = Twist()
    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel

    if (target_angular_vel != 0 or target_linear_vel != 0):
		teleop_pub.publish(twist)


def destination_cb(msg):
    pass

# subscribers
system_vitals_sub = rospy.Subscriber('/vitals_status', RoverStatusMessage, vitals_cb)
image_sub =  rospy.Subscriber('/camera/rgb/image_rect_color/compressed', CompressedImage, im_cb)
# publishers
rover_status_pub = rospy.Publisher('/system_status', UInt8, queue_size=1)
teleop_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

# web topics
web_teleop_sub = rospy.Subscriber('/web/teleop', UInt8, teleop_cb)
web_destination_sub = rospy.Subscriber('/web/destination', String, destination_cb)
web_camera_pub = rospy.Publisher('/web/camera', CompressedImage, queue_size=1)
web_state_pub = rospy.Publisher('/web/state', String, queue_size=1)

pygame.mixer.init()


rospy.init_node('rover_controller')
# nav_controller = actionlib.SimpleActionClient('navigation_controller', NavigationControllerAction)
# print("asking for server")
# nav_controller.wait_for_server()
# print("server up")
# test_pose = Pose()
# test_pose.position.x,test_pose.position.y,test_pose.position.z = [random.randrange(0,100) for _ in range(3)]
# test_pose.orientation.x,test_pose.orientation.y,test_pose.orientation.z, \
#     test_pose.orientation.w = [random.randrange(0,100) for _ in range(4)]
# goal = NavigationControllerGoal()
# goal.goal_pose = test_pose

# nav_controller.send_goal(goal, feedback_cb=nav_cb)

rospy.spin()
