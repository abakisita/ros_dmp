import numpy as np
import yaml
import seaborn
import os.path
import pydmps

class roll_dmp():

    def __init__(self, file_name, sn_dmps=5, n_bfs=50 ):

        weight = self.load_weights(file_name)
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=5, n_bfs=50, ay=np.ones(5)*10.0, w=weight)

    def roll(self, o_goal, o_y0):

        return self.dmp.rollout(goal=o_goal, y0=o_y0, tau=0.26)

    def load_weights(self, file_name):

        with open(file_name) as f:
            loadeddict = yaml.load(f)
        joint_1 = loadeddict.get('joint_1')
        joint_2 = loadeddict.get('joint_2')
        joint_3 = loadeddict.get('joint_3')
        joint_4 = loadeddict.get('joint_4')
        joint_5 = loadeddict.get('joint_5')
        joint_1 = np.array(joint_1)
        joint_2 = np.array(joint_2)
        joint_3 = np.array(joint_3)
        joint_4 = np.array(joint_4)
        joint_5 = np.array(joint_5)

        weights = joint_1
        weights = np.vstack((weights, joint_2))
        weights = np.vstack((weights, joint_3))
        weights = np.vstack((weights, joint_4))
        weights = np.vstack((weights, joint_5))

        return weights
"""
Test code
"""

if __name__ == "__main__":
    path = '/home/abhishek/r_and_d/ros_i/src/ros_dmp/data/weights/default.yaml'
    trajectory_path = '/home/abhishek/r_and_d/ros_i/src/ros_dmp/data/'
    with open(trajectory_path + 'trajectory_joint.yaml') as f:
        loadeddict = yaml.load(f)

    positions = loadeddict.get('positions')
    velocities = loadeddict.get('velocities')
    accelerations = loadeddict.get('accelerations')
    positions = np.array(positions).T
    velocities = np.array(velocities).T
    accelerations = np.array(accelerations).T
    
    o_pos = positions.copy()
    o_goal = o_pos[:,-1]
    o_y0 = o_pos[:,0]
    roll = roll_dmp(path)
    roll.roll(o_goal, o_y0)
