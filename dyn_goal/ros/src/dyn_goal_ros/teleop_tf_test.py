#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Header
from dyn_goal.msg import dyn_goal_msg

import sys, select, termios, tty
# Because of transformations
import tf_conversions
import tf

import tf2_ros
import geometry_msgs.msg
import termios
import tty
import sys
from select import select

from mbot_perception_msgs.msg import RecognizedObject3DList, RecognizedObject3D

import threading

msg = """
Reading from keyboard
---------------------------
Use the following keys to move the track.
   w
a     d
   s

k key to quit, p key to enable/disable dyn_goal

>>
"""
key = None
ORIGIN_TF = "map"
STEP = 0.1
TARGET_FRAME = "tracked_person_off"

def getKey(settings, timeout):
    key = None

    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        text = sys.stdin.read(1)
    else:
        text = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    if text == "w":
        key = [STEP,0]
    elif text == "s":
        key = [-STEP,0]
    elif text == "a":
        key = [0,STEP]
    elif text == "d":
        key = [0,-STEP]
    elif text == "k":
        return 3
    elif text == "p":
        return 5
    return key

# def getKey():
#     key = None
#     text = input(">>")



#     return key

if __name__=="__main__":
    rospy.init_node('key_teleop')

    br = tf.TransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()

    pub_control = rospy.Publisher("/people_follower/move_base_simple/dyn_goal", dyn_goal_msg, queue_size=5)
    pub_localizer = rospy.Publisher("/generic_localizer/localized_objects_tracker", RecognizedObject3DList, queue_size=5)
    activated = False
    dyn_goal_tf = "tracked_person"
    origin_tf = ORIGIN_TF
    dist = 1.2

    settings = termios.tcgetattr(sys.stdin)
    timeout = 0.1

    try:
        print(msg)

        coord = Point(x=5,y=-4,z=0)
        robot = Point(x=0,y=0,z=0)
        rate = rospy.Rate(10.0)

        vertical_step = 0.5
        horizontal_step = 0.5

        while not rospy.is_shutdown():
            key = getKey(settings, timeout)
            if key == 3:
                break
            elif key == 5:
                activated = not activated
                pub_control.publish(dyn_goal_msg(activated=activated,dyn_goal_tf=dyn_goal_tf,origin_tf=origin_tf,dist=dist))
            elif key is not None:
                new_coord = Point()
                new_coord.x = coord.x + vertical_step*key[0]
                new_coord.y = coord.y + horizontal_step*key[1]
                new_coord.z = coord.z

                coord = new_coord


                #Update tf from robot to coord
                br.sendTransform((coord.x, coord.y, coord.z),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         TARGET_FRAME,
                         ORIGIN_TF)

                key = None
            else:
                br.sendTransform((coord.x, coord.y, coord.z),
                                (0.0, 0.0, 0.0, 1.0),
                                rospy.Time.now(),
                                TARGET_FRAME,
                                ORIGIN_TF)
                new_localization = RecognizedObject3DList()
                new_localization.header = Header(stamp = rospy.Time.now() , frame_id='map')
                new_localization.image_header = new_localization.header

                new_object = RecognizedObject3D()
                new_object.class_name = 'person'
                new_object.confidence = 0.9
                new_object.pose.position.x = coord.x
                new_object.pose.position.y = coord.y
                new_object.pose.position.z = coord.z
                new_object.pose.orientation.x = 0
                new_object.pose.orientation.y = 0
                new_object.pose.orientation.z = 0
                new_object.pose.orientation.w = 1
                new_localization.objects = [new_object]
                pub_localizer.publish(new_localization)



    except Exception as e:
        print(e)
