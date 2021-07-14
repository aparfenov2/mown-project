#!/usr/bin/env python  
import roslib
import math
import tf
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        x,y,z = trans
        z = -z
        x,y = 0,0
        trans = [x,y,z]

        r,p,y = tf.transformations.euler_from_quaternion(rot)
        r,p = 0, 0
        rot = tf.transformations.quaternion_from_euler(r,p,y)

        br.sendTransform(trans, rot, rospy.Time.now(), "base_footprint", "base_link")
        rate.sleep()
