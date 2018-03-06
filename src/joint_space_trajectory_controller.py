#!/usr/bin/env python
import rospy
import numpy as np
import tf
import yaml

import geometry_msgs
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint


def load_trajectory():
    path = "../data/"

    with open(path + 'trajectory_joint.yaml') as f:
        loadeddict = yaml.load(f)
    positions = loadeddict.get('positions')
    velocities = loadeddict.get('velocities')
    accelerations = loadeddict.get('accelerations')
    positions = np.array(positions)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)


    return positions, velocities, accelerations


if __name__ == '__main__':

    rospy.init_node('joint_space_trajectory_controller')
    topic_name = "/arm_1/arm_controller/follow_joint_trajectory/goal"
    traj_publisher = rospy.Publisher(topic_name, FollowJointTrajectoryActionGoal, queue_size=1)

    traj = load_trajectory()

    pos = traj[0]
    vel = traj[1]
    acc = traj[2]

    message = FollowJointTrajectoryActionGoal()

    message.header.seq = 1
    message.goal.trajectory.header.frame_id = "/base_footprint"
    message.goal.trajectory.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

    p = JointTrajectoryPoint()

    for i in range(pos.shape[0]):
        p = JointTrajectoryPoint()
        p.positions = pos[i]
        p.velocities = vel[i]
        p.accelerations = acc[i]
        message.goal.trajectory.points.append(p)

    print(message)
    traj_publisher.publish(message)


