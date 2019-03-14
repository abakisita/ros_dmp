#!/usr/bin/env python
import rospy
import numpy as np
from ros_dmp import RollDmp
from ros_dmp.srv import *
from ros_dmp.msg import *
import tf

class GenerateMotion:

    def __init__(self):

        rospy.init_node("generate_motion_service_node")
        rospy.Service("generate_motion_service", generate_motion)

        # Publishers
        self.trajectory_pub = rospy.Publisher('~cartesian_trajectory', CartesianTrajectory)

    def generate_motion(self, req):

        # Initial pose
        rpy = tf.transformations.euler_from_quaternion([req.initial_pose.orientation.x, req.initial_pose.orientation.y,
                                                                    req.initial_pose.orientation.z, req.initial_pose.orientation.w])
        initial_pose = np.array([req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z,
                            rpy[0], rpy[1], rpy[2]])
        
        # Goal Pose
        rpy = tf.transformations.euler_from_quaternion([req.goal_pose.orientation.x, req.goal_pose.orientation.y,
                                                                    req.goal_pose.orientation.z, req.goal_pose.orientation.w])
        goal_pose = np.array([req.goal_pose.position.x, req.goal_pose.position.y, req.goal_pose.position.z,
                            rpy[0], rpy[1], rpy[2]])
        dmp = RollDmp(req.dmp_name.data, req.dt)
        pos, vel, acc = dmp.roll(goal_pose, initial_pose, req.tau)

        # Publish cartesian trajectory
        cartesian_trajectory = CartesianTrajectory()
        cartesian_trajectory.header.frame_id = "base_link"
        for i in range(pos.shape[1]):
            x, y, z, w = tf.transformations.quaternion_from_euler(pos[3, i], pos[4, i], pos[5, i])
            cartesian_state = CartesianState()
            cartesian_state.pose.position.x = pos[0, i]
            cartesian_state.pose.position.x = pos[1, i]
            cartesian_state.pose.position.x = pos[2, i]
            cartesian_state.pose.orientation.x = x
            cartesian_state.pose.orientation.y = y
            cartesian_state.pose.orientation.z = z
            cartesian_state.pose.orientation.w = w

            cartesian_state.vel.linear.x = vel[0, i]
            cartesian_state.vel.linear.x = vel[1, i]
            cartesian_state.vel.linear.x = vel[2, i]
            cartesian_state.vel.angular.x = vel[3, i]
            cartesian_state.vel.angular.x = vel[4, i]
            cartesian_state.vel.angular.x = vel[5, i]

            cartesian_state.acc.linear.x = acc[0, i]
            cartesian_state.acc.linear.x = acc[1, i]
            cartesian_state.acc.linear.x = acc[2, i]
            cartesian_state.acc.angular.x = acc[3, i]
            cartesian_state.acc.angular.x = acc[4, i]
            cartesian_state.acc.angular.x = acc[5, i]
            cartesian_trajectory.cartesian_state.append(cartesian_state)
            
if __name__ == "__main__":
    

