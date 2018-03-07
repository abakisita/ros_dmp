#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import TwistStamped
import control_msgs.msg
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal
import thread, sys, signal
import yaml
import time
import math
import actionlib
import copy
position = []
velocity = []
acceleration = []

count = 0

message = FollowJointTrajectoryGoal()

def callback(data):
    global message
    global position
    global velocity
    global acceleration
    global count

    if count == 0:
        print("trajectory recorded once")
        message.trajectory = copy.deepcopy(data.goal.trajectory)
    
    count += 1

def main():
    global message
    rospy.init_node('test_node')

    print("node initiated")
    trajectory_controller = "/arm_1/arm_controller/follow_joint_trajectory"
    topic_name = "/arm_1/arm_controller/follow_joint_trajectory/goal"
    joint_space_sub =  rospy.Subscriber(topic_name, FollowJointTrajectoryActionGoal, callback)

    ip = raw_input()

    print("input recieved sending ..........")
    client = actionlib.SimpleActionClient(
            trajectory_controller, control_msgs.msg.FollowJointTrajectoryAction
        )
    rospy.loginfo("Waiting for '{0}' server".format(trajectory_controller))
    client.wait_for_server()
    rospy.loginfo("Found server '{0}'".format(trajectory_controller))
    client.send_goal(message)


    rospy.spin()

if __name__ == '__main__':
    main()