#!/usr/bin/env python
import numpy as np
import yaml
import matplotlib.pyplot as plt
import seaborn

import pydmps
import pydmps.dmp_discrete

path = "../data/"

with open(path + 'trajectory_joint.yaml') as f:
    loadeddict = yaml.load(f)
positions = loadeddict.get('positions')
velocities = loadeddict.get('velocities')
accelerations = loadeddict.get('accelerations')
positions = np.array(positions)
velocities = np.array(velocities)
accelerations = np.array(accelerations)

positions = positions.T
velocities = velocities.T
accelerations = accelerations.T
o_pos = positions.copy()
o_goal = o_pos[:,-1]
o_y0 = o_pos[:,0]


positions -= positions[:,0][:,None]

dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=5, n_bfs=50, ay=np.ones(5)*10.0)
y_track = []
dy_track = []
ddy_track = []

dmp.imitate_path(y_des=positions, plot=False)
goal = positions[:,-1]
y0 = positions[:,0]
y_track, dy_track, ddy_track = dmp.rollout(goal=o_goal, y0=o_y0, tau=0.26)

t = np.linspace(0,5,39)
t_ = np.linspace(0,5,y_track.shape[0])
print("y0 is", o_goal)
'''
plt.plot(t_, dy_track[:,0])
plt.plot(t, velocities[0])
plt.show()
plt.plot(t_, dy_track[:,1])
plt.plot(t, velocities[1])
plt.show()
plt.plot(t_, dy_track[:,2])
plt.plot(t, velocities[2])
plt.show()
plt.plot(t_, dy_track[:,3])
plt.plot(t, velocities[3])
plt.show()
plt.plot(t_, dy_track[:,4])
plt.plot(t, velocities[4])
'''
plt.plot(t_, y_track[:,0])
plt.plot(t, o_pos[0])
plt.show()
plt.plot(t_, y_track[:,1])
plt.plot(t, o_pos[1])
plt.show()
plt.plot(t_, y_track[:,2])
plt.plot(t, o_pos[2])
plt.show()
plt.plot(t_, y_track[:,3])
plt.plot(t, o_pos[3])
plt.show()
plt.plot(t_, y_track[:,4])
plt.plot(t, o_pos[4])

plt.show()

data = {'positions': np.asarray(y_track).tolist(), 'velocities': np.asarray(dy_track).tolist(),
        'accelerations': np.asarray(ddy_track).tolist()}
file = path + "learned_trajectory.yaml"
with open(file, "w") as f:
    yaml.dump(data, f)

print("Trajectory saved successfully !!!!")


