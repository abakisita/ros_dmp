Test report 07.07.2018
======================

|                                      |                                       |
|--------------------------------------|---------------------------------------|
| Date & Time                          | 07.07.2018, 13:30-18:00               |
| Tester                               | Abhishek Padalkar                     |
| Software versions                    | ROS Kinetic                            |
|                                      | OpenCV 3.1.0                          |
| Hardware configuration               | Toyota HSR                         |
|                                      | ASUS Xtion Pro Live                   |
| Tested feature                       | Acquisition and repetion of a sequence of dynamic motion primitives that are partially outside of the robot's dexterous workspace |
| Test setup / environment             | C069 lab, HBRS, RoboCup@Home arena    |
|                                      | lab floor (ground level)              |


## Test procedure

### On the robot

1. Switch on Lucy.
2. On the `lucy` account launch the `move_arm` action: `roslaunch mas_hsr_move_arm_action move_arm.launch` (This will also launch `arm_cartesian_control`
    `joint_velocity_control` and `moveit`).

### On an external machine

1. Export the `ROS_MASTER_URI`: `export ROS_MASTER_URI=http://190.168.50.201:11311`
2. Run the script *demonstrated_trajectory_recorder.py* in order to record a trajectory: `rosrun demonstrated_trajectory_recorder demonstrated_trajectory_recorder.py`
3. In the command prompt, press ENTER to start recording; this will open a window that shows the current view of the camera
4. Demonstrate a trajectory by moving the aruco marker board
5. When done demonstrating, press q in the window and write a trajectory name in the command prompt
6. Repeat 3-5 for both trajectories
7. Copy both of the recorded file to `ros_dmp/data/recorded_trajectories/02_08`
8. Run the script `ros_dmp/src/learn_motion_primitive.py` twice to learn the weights of the two motion primitives; the weights will be saved to `ros_dmp/data/weights/weights_<trajectory_name>.yaml`.
9. For safety reasons, move the robot to open space before repeating any demonstrated trajectories.
10. To repeat the demonstrated trajectories, change the initial and goal positions in `ros_dmp/src/test.py` and then run `test.py`. If multiple goal poses are specified, the robot will go to each one of those; however, each pose will first be repeated a given number of times. After finishing each trial, press ENTER to go to the next trial. The data for each trial (both the planned and executed trajectories) are saved to a specified location.


## Test assumptions

* There is an obstacle in front of the robot (a small table)
* The camera used for recording the trajectory is calibrated
* The camera frame rate is low (~5fps), so the trajectory has to be demonstrated slowly
* The test environment is static
* Trajectories are recorded in the base link frame
* The joint encoders are working properly and their measurements are considered ground truth values (there is no external robot pose observer, so the encoder measurements are used for measuring the position of the manipulator)


## Test objectives

* Acquiring demonstrated trajectories (a straight-line motion and an inverse parabolic curve) which are partially outside of the robot's dexterous workspace
* Repeating the demonstrated trajectories in a sequential manner in order to execute a pick-and-place task (picking an object from a table and then placing it on another higher table).

## Test parameters

* Number of basis functions: 50
* tau = 30
* Number of trials: 10

## Observations

* Since the demonstrated trajectory has initial and final z values that are practically equal to each other, varying z creates infeasible resulting trajectories; this primitive is thus limited to scenarios in which the initial and final z position are the same (e.g. moving an object from one side of a table to another)