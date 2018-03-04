#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import TwistStamped
import control_msgs
from control_msgs.msg import FollowJointTrajectoryActionGoal
import thread, sys, signal
import yaml


position = []
velocity = []
acceleration = []

count = 0


def callback(data):
    global position
    global velocity
    global acceleration
    global count

    count += 1
    point_array = data.goal.trajectory.points
    for p in point_array:
        position.append(p.positions)
        velocity.append(p.velocities)
        acceleration.append(p.accelerations)


def main():
    global trajectory
    global trajectory_rot
    rospy.init_node('trajectory_recorder_joint_space')

    print("node initiated")

    topic_name = "/arm_1/arm_controller/follow_joint_trajectory/goal"



    joint_space_sub =  rospy.Subscriber(topic_name, FollowJointTrajectoryActionGoal, callback)

    while count == 0:
        pass
    save_trajectory()

def save_trajectory():
    path = "../data/"


    global position
    global velocity
    global acceleration
    data = {'positions': np.asarray(position).tolist(), 'velocitys': np.asarray(velocity).tolist(),
            'accelerations': np.asarray(acceleration).tolist()}
    file = path + "trajectory_joint.yaml"
    with open(file, "w") as f:
        yaml.dump(data, f)

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
