import numpy as np
import yaml
import seaborn
import os.path
import pydmps

class roll_dmp():

    def __init__(self, file_name, n_dmps=6, n_bfs=50 ):

        weight = self.load_weights(file_name)
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs, dt=0.001, ay=np.ones(n_dmps)*10.0, w=weight)

    def roll(self, goal, initial_pos, tau):

        self.pos, self.vel, self.acc = self.dmp.rollout(goal=goal, y0=initial_pos, tau=tau)
        return self.pos, self.vel, self.acc

    def load_weights(self, file_name):

        with open(file_name) as f:
            loadeddict = yaml.load(f)
        x = loadeddict.get('x')
        y = loadeddict.get('y')
        z = loadeddict.get('z')
        roll = loadeddict.get('roll')
        pitch = loadeddict.get('pitch')
        yaw = loadeddict.get('yaw')

        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        roll = np.array(roll)
        pitch = np.array(pitch)
        yaw = np.array(yaw)


        weights = x
        weights = np.vstack((weights, y))
        weights = np.vstack((weights, z))
        weights = np.vstack((weights, roll))
        weights = np.vstack((weights, pitch))
        weights = np.vstack((weights, yaw))

        return weights

    


"""
Test code
"""

if __name__ == "__main__":
    path = '/home/abhishek/r_and_d/ros_i/src/ros_dmp/data/weights/weights_s01.yaml'
    trajectory_path = '/home/abhishek/r_and_d/ros_i/src/ros_dmp/data/recorded_trajecories/'
    with open(trajectory_path + 'trajectory_joint.yaml') as f:
        loadeddict = yaml.load(f)

    linear_trajectory = loadeddict.get('linear_trajectory')
    rotational_trajectory = loadeddict.get('rotational_trajectory')
    
    linear_init = linear_trajectory[0]
    rotational_init = rotational_trajectory[0]
    linear_goal = linear_trajectory[0]
    rotational_ = rotational_trajectory[0]

    roll = roll_dmp(path)
    roll.roll(o_goal, o_y0)
