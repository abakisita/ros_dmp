#!/usr/bin/env python
import numpy as np
import yaml
import seaborn
from os.path import join
import pydmps


import rospy 
import geometry_msgs 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class ros_dmp:
    def __init__(self, trajectory_file):
        

        self.path_pub = rospy.Publisher("/dmp_executor/debug_path", Path, queue_size=1)
        rospy.sleep(2)
        # Path to store weight and load learning data
        #abs_path = os.path.abspath()
        trajectory_path = "../data/recorded_trajectories/old_trajectories"
        self.weight_path = "../data/weights/"

        # Loading trajectory to learn
        with open(join(trajectory_path, trajectory_file)) as f:
            loadeddict = yaml.load(f)

        linear_trajectory = loadeddict.get('linear_trajectory')
        rotational_trajectory = loadeddict.get('rotational_trajectory')
        self.linear_trajectory = np.array(linear_trajectory)
        self.rotational_trajectory = np.array(rotational_trajectory)

        self.positions = np.hstack((self.linear_trajectory, self.rotational_trajectory))
        self.positions = self.positions.T
        self.o_pos = self.positions.copy()
        self.o_goal = self.o_pos[:,-1]
        self.o_y0 = self.o_pos[:,0]
        print self.o_goal, self.o_y0
        # Removing bias from the data. (Start position is zero now)
        self.positions -= self.positions[:,0][:,None]

        # Initiating DMP
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=6, n_bfs=1500, ay=np.ones(6)*10.0)
        


    def learn_dmp(self, file_name="default"):
        """
        This function learns dmp weights and stores to desired file
        """

        weights = self.dmp.imitate_path(y_des=self.positions, plot=False)
        self.pos, self.vel, self.acc = self.dmp.rollout(goal=self.o_goal, y0=self.o_y0)

        data = {'x': np.asarray(weights[0,:]).tolist(), 'y': np.asarray(weights[1,:]).tolist(),
            'z': np.asarray(weights[2,:]).tolist(), 'roll': np.asarray(weights[3,:]).tolist(), 
            'pitch': np.asarray(weights[4,:]).tolist(), 'yaw': np.asarray(weights[5,:]).tolist()}
        file = join(self.weight_path, file_name)
        with open(file, "w") as f:
            yaml.dump(data, f)

            
        path = Path()
        path.header.frame_id = "/base_link"
        for itr in range(self.pos.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = self.pos[itr,0]
            pose_stamped.pose.position.y = self.pos[itr,1]
            pose_stamped.pose.position.z = self.pos[itr,2]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)

'''
Test code
'''
if __name__ == "__main__":
    

    rospy.init_node("learn_motion_primitive")
    trajectory_file = raw_input('Enter the trajectory file name: ')
    dmp = ros_dmp(trajectory_file=trajectory_file)
    weights_file = "weights_" + trajectory_file
    dmp.learn_dmp(file_name=weights_file)
    