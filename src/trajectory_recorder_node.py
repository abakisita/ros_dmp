#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import TwistStamped
import thread, sys, signal
import yaml

time_instance = []
path = "../data/"
run_flag = True
trajectory = np.zeros((3))
trajectory_rot = np.zeros((3))
def check_user_input():
    while True:
        ip = raw_input()
        print "here we go"
        if ip == 's':
            run_flag = False
            break



def main():
    global trajectory
    global trajectory_rot
    rospy.init_node('trajectory_recorder')

    print("node initiated")
    tf_listener = tf.TransformListener()
    while True:
        try:
            (init_pos,rot) = tf_listener.lookupTransform('base_link', 'arm_link_5', rospy.Time(0))
            print("trying")
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed")
            continue
    init_pos = np.array(init_pos)
    rot = np.array(rot)

    last_point = init_pos.copy()

    print("Initial pose recorded, Ready to GO .... Start motion")

    trajectory = init_pos[np.newaxis]
    rot = tf.transformations.euler_from_quaternion(rot)
    rot = np.array(rot)
    trajectory_rot = rot[np.newaxis]
    print(rot)
    init_time = 0.0

    while True:
        try:
            (pos,rot) = tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
            if np.linalg.norm((init_pos - np.array(pos))) > 0.00001 :
                init_time = rospy.get_time()
                time_instance.append(init_time)
                trajectory = np.vstack((trajectory, np.array(pos)))
                rot = tf.transformations.euler_from_quaternion(rot)
                trajectory_rot = np.vstack((trajectory_rot, np.array(rot)))
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    global time_instance

    while run_flag:
        try:
            (pos,rot) = tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        if last_point[0] == pos[0] and last_point[1] == pos[1] and last_point[2] == pos[2]:
            pass
        else:
            time_instance.append(rospy.get_time() - init_time)
            trajectory = np.vstack((trajectory, np.array(pos)))
            rot = tf.transformations.euler_from_quaternion(rot)
            trajectory_rot = np.vstack((trajectory_rot, np.array(rot)))

            last_point = np.array(pos)
            print(rot)

def save_trajectory():
    global trajectory
    global trajectory_rot
    global time_instance
    data = {'translation': np.asarray(trajectory).tolist(), 'rotation': np.asarray(trajectory_rot).tolist(),
            'time': np.asarray(time_instance).tolist()}
    file = path + "trajectory.yaml"
    with open(file, "w") as f:
        yaml.dump(data, f)
    print("time instance ")
    print(time_instance)
    print("Trajectory recorded successfully !!!!")


def signal_handler(signal, frame):
    save_trajectory()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        run_flag = False
        print(run_flag)
        save_trajectory()
