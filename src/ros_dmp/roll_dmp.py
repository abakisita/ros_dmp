import numpy as np
import yaml
import os.path
import pydmps

class RollDmp():

    def __init__(self, file_name, dt):

        weights = self.load_weights(file_name)
        n_dmps = weights.shape[0]
        n_bfs = weights.shape[1]
        self.dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs,
                                                     dt=dt, ay=None, w=weights)

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

        weights = np.array(x)
        weights = np.vstack((weights, np.array(y)))
        weights = np.vstack((weights, np.array(z)))
        weights = np.vstack((weights, np.array(roll)))
        weights = np.vstack((weights, np.array(pitch)))
        weights = np.vstack((weights, np.array(yaw)))

        return weights

