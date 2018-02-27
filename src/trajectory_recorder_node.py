#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import TwistStamped
import thread
import yaml


path = "../data/"
run_flag = True
def check_user_input():
    while True:
        ip = raw_input()
        if ip == 's':
            run_flag = False
            break

if __name__ == '__main__':

    tf_listener = tf.TransformListener()
    while True:
        try:
            (init_pos,rot) = tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    init_pos = np.array(init_pos)
    rot = np.array(rot)


    print("Initial pose recorded, Ready to GO .... Start motion")

    trajectory = init_pos[np.newaxis]

    while True:
        try:
            (pos,rot) = tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
            if np.linalg.norm((init_pos - np.array(pos))) > 0.001 :
                trajectory = np.vstack((trajectory, np.array(pos)))
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    thread.start_new_thread(check_user_input)

    while run_flag:
        try:
            (pos,rot) = tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        trajectory = np.vstack((trajectory, np.array(pos)))

    data = {'trajectory': np.asarray(trajectory).tolist()}
    file = path + "trajectory.yaml"
    with open(file, "w") as f:
        yaml.dump(data, f)

    print("Trajectory recorded successfully !!!!")