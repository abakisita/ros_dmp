#!/usr/bin/env python
import yaml
import pydmps
import rospy
import sys
import tf
import std_msgs
import numpy as np
from os.path import join
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ros_dmp.srv import *


class LearnDmp:
    def __init__(self):
        '''Ros interface for learning DMP

        Initializes the learn DMP service
        '''
        rospy.init_node("learn_dynamic_motion_primitive_service")
        service_ = rospy.Service('learn_dynamic_motion_primitive_service',
                                 LearnDMP, self.learn_dmp_handler)
        rospy.loginfo("Started learn DMP service")

        # Publishers
        self.imitated_path_pub = rospy.Publisher("~imitated_path", Path, queue_size=1)
        self.demonstrated_path_pub = rospy.Publisher("~demonstrated_path", Path, queue_size=1)

        # Parameters
        self.weights_file_path = rospy.get_param('~weights_file_path', '../../data/weights/')
        loop_rate = rospy.get_param('~loop_rate')

        self.result = ""
        r = rospy.Rate(loop_rate)
        rospy.spin()

    def learn_dmp_handler(self, req):
        '''Handler for client request

        req: service request msg
        '''
        rospy.loginfo("Recieved request to learn a motion primitive")
        trajectory = np.zeros((6, len(req.poses)))
        rospy.loginfo("Learning motion primitive " + req.dmp_name)
        for i in range(len(req.poses)):
            rpy = tf.transformations.euler_from_quaternion([req.poses[i].orientation.x,
                                                            req.poses[i].orientation.y,
                                                            req.poses[i].orientation.z,
                                                            req.poses[i].orientation.w])
            trajectory[:, i] = [req.poses[i].position.x, req.poses[i].position.y,
                                req.poses[i].position.z, rpy[0], rpy[1], rpy[2]]
        self.learn_dmp(trajectory, req.output_weight_file_name, req.n_dmps, req.n_bfs)
        rospy.loginfo("Successfully learned the motion primitive")
        # Return response
        response = LearnDMPResponse()
        response.result = self.result
        return response

    def learn_dmp(self, trajectory, file_name, n_dmps=6, n_bfs=50):
        """This function learns dmp weights and stores them in desired file

        trajectory: Matrix containing trajectory
        file_name: Name of file in which weights will be stored
        n_dmps: Number of dimmensions (6 default for cartesian trajectory)
        n_bfs: Number of basis functions to be used
        """

        demonstrated_trajectory = trajectory.copy()
        demonstrated_goal_pose = demonstrated_trajectory[:, -1]
        demonstrated_initial_pose = demonstrated_trajectory[:, 0]

        # Removing bias from the data. (Start position is zero now)
        trajectory -= trajectory[:, 0][:, None]

        # Initiating DMP
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs, ay=None)

        # Learn weights
        weights = self.dmp.imitate_path(y_des=trajectory)

        #save weights to desired file
        data = {'x': np.asarray(weights[0, :]).tolist(), 'y': np.asarray(weights[1, :]).tolist(),
                'z': np.asarray(weights[2, :]).tolist(), 'roll': np.asarray(weights[3, :]).tolist(),
                'pitch': np.asarray(weights[4, :]).tolist(),
                'yaw': np.asarray(weights[5, :]).tolist()}
        file = join(self.weights_file_path, file_name)
        try:
            with open(file, "a+") as f:
                yaml.dump(data, f)
            self.result = "success"
        except:
            rospy.logerr("Cannot save weight file. Check if the directory of the weight file exists. Related parameter can be found in launch file.")
            self.result = "failed"

        # Imitate the same path as demonstrated
        pos, vel, acc = self.dmp.rollout(goal=demonstrated_goal_pose, y0=demonstrated_initial_pose)

        # Publish Imitated Path
        imitated_path = Path()
        imitated_path.header.frame_id = "/base_link"
        for itr in range(pos.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pos[itr, 0]
            pose_stamped.pose.position.y = pos[itr, 1]
            pose_stamped.pose.position.z = pos[itr, 2]
            imitated_path.poses.append(pose_stamped)
        self.imitated_path_pub.publish(imitated_path)

        # Publish Demonstrated Path
        demonstrated_path = Path()
        demonstrated_path.header.frame_id = "/base_link"
        for itr in range(demonstrated_trajectory.shape[1]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = demonstrated_trajectory[0, itr]
            pose_stamped.pose.position.y = demonstrated_trajectory[1, itr]
            pose_stamped.pose.position.z = demonstrated_trajectory[2, itr]
            demonstrated_path.poses.append(pose_stamped)
        self.demonstrated_path_pub.publish(demonstrated_path)
