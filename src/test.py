#!/usr/bin/env python
import rospy
import numpy as np
import geometry_msgs
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import control_msgs.msg
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal
import thread, sys, signal
import yaml
import time
import math
import actionlib
import copy
import mcr_manipulation_utils_ros.kinematics as kinematics
from learn_motion_primitive import ros_dmp
from trajectory_msgs.msg import JointTrajectoryPoint


def main():

    rospy.init_node('test_node')

    trajectory_controller = "/arm_1/arm_controller/follow_joint_trajectory"
    group_name = "arm_1"
    client = actionlib.SimpleActionClient(
            trajectory_controller, control_msgs.msg.FollowJointTrajectoryAction
        )
    rospy.loginfo("Waiting for '{0}' server".format(trajectory_controller))
    client.wait_for_server()
    rospy.loginfo("Found server '{0}'".format(trajectory_controller))
    k_solver = kinematics.Kinematics(group_name)
    pose = PoseStamped()

    joints = k_solver.inverse_kinematics(pose)
    
    print(joints)

    ros_dmp_ = ros_dmp()
    ros_dmp_.learn_dmp()
    pos, vel, acc = ros_dmp_.roll()
    message = FollowJointTrajectoryGoal()
    message.trajectory.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
    for i in range(pos.shape[0]):
        p = JointTrajectoryPoint()
        p.positions = pos[i]
        p.velocities = vel[i]
        p.accelerations = acc[i]
        message.trajectory.points.append(p)
    
    client.send_goal(message)






if __name__ == '__main__':
    main()
    rospy.spin()