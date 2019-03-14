#!/usr/bin/env python
import numpy as np 
import rospy
from geometry_msgs.msg import Pose
from ros_dmp.srv import *

if __name__ == "__main__":

    rospy.init_node('learn_dmp_service_test_client')
    req = LearnDMPRequest()

    # Generating a hypothetical trajectory
    x = np.linspace(0, 5)
    y = np.linspace(0, 5)
    z = np.zeros(10)
    z = np.hstack((z, np.ones(30)))
    z = np.hstack((z, np.ones(10)*0.5))
    o_x = np.linspace(0, 1)
    o_y = np.linspace(0, 1)
    o_z = np.linspace(0, 1)
    o_w = np.linspace(0, 1)

    req.header.frame_id = 'base_link'
    req.output_weight_file_name.data = 'example.yaml'
    req.dmp_name.data = 'square_wave'
    req.header.stamp = rospy.Time.now()
    req.n_bfs = 500
    req.n_dmps = 6
    for i in range(x.shape[0]):
        pose = Pose()
        pose.position.x = x[i]
        pose.position.y = y[i]
        pose.position.z = z[i]
        pose.orientation.x = o_x[i]
        pose.orientation.y = o_y[i]
        pose.orientation.z = o_z[i]
        pose.orientation.w = o_w[i]
        req.poses.append(pose)
    try:
        service_client = rospy.ServiceProxy('/learn_dynamic_motion_primitive_service', LearnDMP)
        rospy.loginfo(service_client(req))
    except :
        rospy.loginfo("Service call failed")
    