#!/usr/bin/env python
import numpy as np
import yaml
import seaborn
import os.path
import pydmps


class ros_dmp:
    def __init__(self):
        trajectory_path = "../data/"
        self.weight_path = "../data/weights/"
        with open(trajectory_path + 'trajectory_joint.yaml') as f:
            loadeddict = yaml.load(f)
        positions = loadeddict.get('positions')
        velocities = loadeddict.get('velocities')
        accelerations = loadeddict.get('accelerations')
        self.positions = np.array(positions).T
        self.velocities = np.array(velocities).T
        self.accelerations = np.array(accelerations).T
        self.o_pos = self.positions.copy()
        self.o_goal = self.o_pos[:,-1]
        self.o_y0 = self.o_pos[:,0]
        self.positions -= self.positions[:,0][:,None]
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=5, n_bfs=50, ay=np.ones(5)*10.0)


    def learn_dmp(self, file_name="default"):
        weights = self.dmp.imitate_path(y_des=self.positions, plot=False)
        data = {'joint_1': np.asarray(weights[0,:]).tolist(), 'joint_2': np.asarray(weights[1,:]).tolist(),
            'joint_3': np.asarray(weights[2,:]).tolist(), 'joint_4': np.asarray(weights[3,:]).tolist(), 
            'joint_5': np.asarray(weights[4,:]).tolist()}
        file = self.weight_path + file_name + ".yaml"
        print weights
        with open(file, "w") as f:
            yaml.dump(data, f)
    
    def roll(self):
        
        y_track, dy_track, ddy_track = self.dmp.rollout(goal=self.o_goal, y0=self.o_y0, tau=0.26)
        return y_track, dy_track, ddy_track


'''
Test code
'''
if __name__ == "__main__":
    
    dmp = ros_dmp()
    dmp.learn_dmp()
    dmp.roll()