import rospy 
import numpy as np 
import sensor_msgs.msg

def laser_scan_filter(msg):
    ranges = msg.ranges[331:632]
    min_angle = msg.angle_min
    ang_incre = msg.angle_increment
    distances = []
    for i in range(len(ranges)):

        distances.append(ranges[i] * np.cos(min_angle + ang_incre * (331 + i)))


    #print len(msg.ranges)
    print min(distances)


if __name__ == "__main__":

    rospy.init_node("laser_filter_test")
    rospy.Subscriber("/hsrb/base_scan", sensor_msgs.msg.LaserScan, laser_scan_filter)

    while not rospy.is_shutdown():
        pass