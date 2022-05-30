#!/usr/bin/env python3

import rospy
from math import *
import numpy as np
from nav_msgs.msg import Odometry
import tf

last_odom = None
pose = [0.0,0.0,0.0]
a1 = 0.0
a2 = 0.0
a3 = 0.0
a4 = 0.0
new_odom_frame = ""
odom_frame = ""


def callback(data):
    global last_odom
    global new_odom_frame
    global odom_frame
    global pose
    global a1
    global a2
    global a3
    global a4

    q = [ data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w ]

    (r, p, theta2) = tf.transformations.euler_from_quaternion(q)

    if(last_odom == None):
        last_odom = data
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y
        pose[2] = theta2
    else:
        dx = data.pose.pose.position.x - last_odom.pose.pose.position.x
        dy = data.pose.pose.position.y - last_odom.pose.pose.position.y
        trans = sqrt(dx*dx + dy*dy)

        q = [ last_odom.pose.pose.orientation.x,
                last_odom.pose.pose.orientation.y,
                last_odom.pose.pose.orientation.z,
                last_odom.pose.pose.orientation.w ]

        (r,p, theta1) = tf.transformations.euler_from_quaternion(q)
        rot1 = atan2(dy, dx) - theta1
        rot2 = theta2-theta1-rot1

        sd_rot1 = a1*abs(rot1) + a2*trans
        sd_rot2 = a1*abs(rot2) + a2*trans
        sd_trans = a3*trans + a4*(abs(rot1) + abs(rot2))

        trans +=  np.random.normal(0,sd_trans*sd_trans)
        rot1 += np.random.normal(0, sd_rot1*sd_rot1)
        rot2 += np.random.normal(0, sd_rot2*sd_rot2)

        pose[0] += trans*cos(theta1+rot1)
        pose[1] += trans*sin(theta1+rot1)
        pose[2] +=  rot1 + rot2
        last_odom = data

    # publish tf
    br = tf.TransformBroadcaster()
    br.sendTransform((pose[0] - data.pose.pose.position.x, pose[1] - data.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, pose[2] - theta2),
                     data.header.stamp,
                     odom_frame,
                     new_odom_frame)

if __name__ == '__main__':
    rospy.init_node('noisy_odometry', anonymous=True)
    
    # alpha 1 is degree/degree
    if rospy.has_param("~alpha1"):
        a1 = rospy.get_param("~alpha1")
    else:
        rospy.logwarn("alpha1 is set to default")
        a1 = 0.05

    # alpha 2 is degree/m
    if rospy.has_param("~alpha2"):
        a2 = rospy.get_param("~alpha2")
    else:
        a2 = 10.0*pi/180.0
        rospy.logwarn("alpha2 is set to default")

    # alpha 3 is m/meter
    if rospy.has_param("~alpha3"):
        a3 = rospy.get_param("~alpha3")
    else:
        a3 = 0.05
        rospy.logwarn("alpha3 is set to default")

    # alpha 4 is m/degree
    if rospy.has_param("~alpha4"):
        a4 = rospy.get_param("~alpha4")
    else:
        a4 = 0.01
        rospy.logwarn("alpha4 is set to default")

    # get odom topic
    if rospy.has_param("~old_odom_topic"):
        odom_topic = rospy.get_param("~old_odom_topic")
    else:
        odom_topic = "/odom"

    # set new odom frame
    if rospy.has_param("~new_odom_frame"):
        new_odom_frame = rospy.get_param("~new_odom_frame")
    else:
        new_odom_frame = "odom_noisy"

    # get odom frame
    if rospy.has_param("~odom_frame"):
        odom_frame = rospy.get_param("~odom_frame")
    else:
        odom_frame = "odom"

    rospy.Subscriber(odom_topic, Odometry, callback)
    rospy.spin()