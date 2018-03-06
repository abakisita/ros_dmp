#!/usr/bin/env python
import rospy
import numpy as np
import tf
import yaml

import geometry_msgs
from geometry_msgs.msg import TwistStamped


def load_trajectory():
    path = "../data/"

    with open(path + 'trajectory.yaml') as f:
        loadeddict = yaml.load(f)
    trajectory_rot = loadeddict.get('rotation')
    trajectory = loadeddict.get('translation')
    trajectory_rot = np.array(trajectory_rot)
    trajectory = np.array(trajectory)
    return trajectory_rot, trajectory



if __name__ == '__main__':
    rospy.init_node('trajectory_controller')
    listener = tf.TransformListener()
    final_pos = np.array([0.524, 0.08, 0.102])
    final_rot = np.array([3.122, 0.292, 2.898])
    init_pos = final_pos
    init_rot = final_rot
    topic_name = "/arm_1/arm_controller/cartesian_velocity_command"
    vel_publisher = rospy.Publisher(topic_name, TwistStamped, queue_size=10)



    while True:
        try:
            (init_pos,rot) = listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    print "fafa", init_pos
    init_pos = np.array(init_pos)
    init_rot = np.array(init_rot)

    path_x = np.linspace(init_pos[0], final_pos[0], 30)
    path_y = np.linspace(init_pos[1], final_pos[1], 30)
    path_z = np.linspace(init_pos[2], final_pos[2], 30)

    final_pos1 = ([0.450, 0.08, 0.102],[0.45, -0.02, 0.102], [0.524, -0.02, 0.102])
    f_old = final_pos

    
    for f in final_pos1:
        
        _x = np.linspace(f_old[0], f[0], 30)
        _y = np.linspace(f_old[1], f[1], 30)
        _z = np.linspace(f_old[2], f[2], 30)
        path_x = np.hstack((path_x,_x))
        path_y = np.hstack((path_y,_y))
        path_z = np.hstack((path_z,_z))
        f_old = f


    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(path_x,path_y,path_z)
    plt.show()
    path = np.hstack((path_x[np.newaxis].T, path_y[np.newaxis].T))
    path = np.hstack((path, path_z[np.newaxis].T))
    print(path)
    distance = np.linalg.norm((np.array(final_pos1[len(final_pos1) - 1]) - init_pos))

    flag = True
    count = 0
    while distance > 0.03:
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        message = TwistStamped()
        current_pos = np.array(trans)
        current_rot = np.array(rot)

        distance = np.linalg.norm((np.array(final_pos1[len(final_pos1) - 1]) - current_pos))
        dist = []
        for i in range(path.shape[0]):
            dist.append(np.linalg.norm((path[i] - current_pos)))
        index =  np.argmin(dist)
        if (index > path.shape[0] - 2):
            break

        if path_x[index + 1] == path_x[index] and path_y[index + 1] == path_y[index] and path_z[index + 1] == path_z[index]:
            index += 1
        vel_x = 20*(path_x[index + 1] - path_x[index]) + 0.2 * (path_x[index + 1] - current_pos[0])
        vel_y = 20*(path_y[index + 1] - path_y[index]) + 0.2 * (path_y[index + 1] - current_pos[1])
        vel_z = 20*(path_z[index + 1] - path_z[index]) + 0.2 * (path_z[index + 1] - current_pos[2])


        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = vel_x
        message.twist.linear.y = vel_y
        message.twist.linear.z = vel_z







        vel_publisher.publish(message)

        count += 1

