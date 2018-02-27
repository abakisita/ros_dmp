import numpy as np
import yaml
import matplotlib.pyplot as plt
import seaborn

import pydmps
import pydmps.dmp_discrete

path = "../data/"

with open(path + 'data.yaml') as f:
    loadeddict = yaml.load(f)
trajectory = loadeddict.get('trajectory')

trajectory = np.array(trajectory)
print(trajectory)
print(trajectory[:,0][:,None])
trajectory -= trajectory[:,0][:,None]
print(trajectory)

dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=3, n_bfs=50, ay=np.ones(2)*10.0)
y_track = []
dy_track = []
ddy_track = []
dmp.imitate_path(y_des=trajectory, plot=True)
goal = np.array([1.5, -1.8])
y0 = np.array([0.2, -0.2])
#y_track, dy_track, ddy_track = dmp.rollout(goal=goal, y0=y0)