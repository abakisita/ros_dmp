#!/usr/bin/env python
import rospy
import numpy as np
import tf
import geometry_msgs
from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    rospy.init_node('trajectory_controller')
    listener = tf.TransformListener()

    final_pos = np.array([0.2539753476936794, -0.020781520241155538, 0.42984685997044936])
    init_pos = final_pos

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

    path_x = np.linspace(init_pos[0], final_pos[0], 100)
    path_y = np.linspace(init_pos[1], final_pos[1], 100)
    path_z = np.linspace(init_pos[2], final_pos[2], 100)

    path = np.hstack((path_x[np.newaxis].T, path_y[np.newaxis].T))
    path = np.hstack((path, path_z[np.newaxis].T))

    distance = np.linalg.norm((final_pos - init_pos))

    flag = True
    count = 0
    while distance > 0.03:
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        message = TwistStamped()
        current_pos = np.array(trans)

        distance = np.linalg.norm((final_pos - current_pos))
        dist = []
        for i in range(path.shape[0]):
            dist.append(np.linalg.norm((path[i] - current_pos)))
        index =  np.argmin(dist)
        if (index > path.shape[0] - 1):
            break
        vel_x = (path_x[index + 1] - path_x[index]) + 0.2 * (path_x[index + 1] - current_pos[0])
        vel_y = (path_y[index + 1] - path_y[index]) + 0.2 * (path_y[index + 1] - current_pos[1])
        vel_z = (path_z[index + 1] - path_z[index]) + 0.2 * (path_z[index + 1] - current_pos[2])

        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = vel_x
        message.twist.linear.y = vel_y
        message.twist.linear.z = vel_z

        vel_publisher.publish(message)

        count += 1

        print message