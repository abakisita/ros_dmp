#!/usr/bin/env python
import rospy
import numpy as np
from ros_dmp import RollDmp
from ros_dmp.srv import *
from ros_dmp.msg import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import tf

class GenerateMotionClass:

    def __init__(self):

        rospy.init_node("generate_motion_service_node")
        rospy.Service("generate_motion_service", GenerateMotion, self.generate_motion)
        rospy.loginfo("Started Motion Generation Service")
        # Publishers
        self.trajectory_pub = rospy.Publisher('~cartesian_trajectory', CartesianTrajectory, queue_size=1)
        self.path_pub = rospy.Publisher('~cartesian_path', Path, queue_size=1)

    def generate_motion(self, req):

        rospy.loginfo("Received motion generation request")
        # Initial pose
        rpy = tf.transformations.euler_from_quaternion([req.initial_pose.pose.orientation.x, req.initial_pose.pose.orientation.y,
                                                                    req.initial_pose.pose.orientation.z, req.initial_pose.pose.orientation.w])
        initial_pose = np.array([req.initial_pose.pose.position.x, req.initial_pose.pose.position.y, req.initial_pose.pose.position.z,
                            rpy[0], rpy[1], rpy[2]])
        
        # Goal Pose
        rpy = tf.transformations.euler_from_quaternion([req.goal_pose.pose.orientation.x, req.goal_pose.pose.orientation.y,
                                                                    req.goal_pose.pose.orientation.z, req.goal_pose.pose.orientation.w])
        goal_pose = np.array([req.goal_pose.pose.position.x, req.goal_pose.pose.position.y, req.goal_pose.pose.position.z,
                            rpy[0], rpy[1], rpy[2]])

        rospy.loginfo("Generating motion for dmp " + req.dmp_name)
        dmp = RollDmp(req.dmp_name, req.dt)
        pos, vel, acc = dmp.roll(goal_pose, initial_pose, req.tau)

        # Publish cartesian trajectory
        cartesian_trajectory = CartesianTrajectory()
        cartesian_trajectory.header.frame_id = "base_link"
        path = Path()
        path.header.frame_id = "base_link"
        for i in range(pos.shape[0]):
            x, y, z, w = tf.transformations.quaternion_from_euler(pos[i, 3], pos[i, 4], pos[i, 5])
            pose = Pose()
            pose.position.x = pos[i, 0]
            pose.position.y = pos[i, 1]
            pose.position.z = pos[i, 2]
            pose.orientation.x = x
            pose.orientation.y = y
            pose.orientation.z = z
            pose.orientation.w = w
            
            cartesian_state = CartesianState()
            cartesian_state.pose = pose
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose

            cartesian_state.vel.linear.x = vel[i, 0]
            cartesian_state.vel.linear.y = vel[i, 1]
            cartesian_state.vel.linear.z = vel[i, 2]
            cartesian_state.vel.angular.x = vel[i, 3]
            cartesian_state.vel.angular.y = vel[i, 4]
            cartesian_state.vel.angular.z = vel[i, 5]

            cartesian_state.acc.linear.x = acc[i, 0]
            cartesian_state.acc.linear.y = acc[i, 1]
            cartesian_state.acc.linear.z = acc[i, 2]
            cartesian_state.acc.angular.x = acc[i, 3]
            cartesian_state.acc.angular.y = acc[i, 4]
            cartesian_state.acc.angular.z = acc[i, 5]
            
            cartesian_trajectory.cartesian_state.append(cartesian_state)
            path.poses.append(pose_stamped)
        
        self.trajectory_pub.publish(cartesian_trajectory)
        self.path_pub.publish(path)
        response = GenerateMotionResponse()
        rospy.loginfo("Motion generated and published on respective toopics")
        response.result = "success"
        response.cart_traj = cartesian_trajectory
        return response

if __name__ == "__main__":
    
    obj = GenerateMotionClass()
    rospy.spin()

