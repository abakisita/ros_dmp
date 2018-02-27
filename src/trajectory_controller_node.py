#!/usr/bin/env python
import rospy
import numpy as np
import tf

if __name__ == '__main__':
    rospy.init_node('trajectory_controller')
    listener = tf.TransformListener()
    while True:
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(trans)