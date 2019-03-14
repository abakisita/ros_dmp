# ros_dmp

## Dynamic Motion Primitives for Learning from Demonstration

This package implements Dynamic Motion Primitives for Learning from Demonstration. This package was developed during the R and D project as part of academics. Detailed report on analysis, implementation and use of this package can be found at https://github.com/abhishek098/r_n_d_report/blob/master/PadalkarAbhishek-%5BRnD%5DReport.pdf . 

Reference : Auke Jan Ijspeert, Jun Nakanishi, Heiko Hoffmann, Peter Pastor, and Stefan Schaal. Dynamical movement primitives: learning attractor models for motor behaviors. Neural computation, 25(2):328{373, 2013}. 

## How to use the package
Clone the repository:

`git clone https://github.com/abhishek098/ros_dmp.git`

Build the package:

Inside the package

`catkin build --this`

#
This package provides two services:
1. Service for learning the DMP
2. Service for genearating motion from already learnt DMP

Geerated motion is published as navigation path on the topic:

`/generate_motion_service_node/cartesian_path`

And as cartesian trajectory on the topic:

`/generate_motion_service_node/cartesian_trajectory`

For more details about using these services, have a look at example clients in `example` folder. 

### Example clients
Running learn DMP and generate motion clients

Launch the DMP simulation launch file in one terminal using following command.

`roslaunch ros_dmp dmp_sim.launch`

Run rviz to visualize learned and demonstrated path published on following topics which can be visualized in rviz. 

`/learn_dmp_service_node/demonstrated_path`

`/learn_dmp_service_node/demonstrated_path`

In another terminal, run learn client script in `example` folder

`python learn_client.py`

This script calls the learn DMP service for imitating a artificial trajectory. 

Observe the imitated and demonstrated trajectories in rviz.

In another terminal, run generate motion client

`python motion_generation_client.py`

Genrated motion can be visualized in rviz on following topic:

`/generate_motion_service_node/cartesian_path`
