#!/usr/bin/python
import numpy as np
import rospy
import roll_dmp
import std_msgs
import geometry_msgs 
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from nav_msgs.msg import Path
import tf
import actionlib
import actionlib_msgs
import moveit_msgs
import moveit_commander
import mcr_manipulation_utils_ros.kinematics as kinematics
import yaml
import time

class dmp_executor():

    def __init__(self, dmp_name, tau):
        
        '''
        rospy.Subscriber("/dmp_executor/event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("/dmp_executor/goal", geometry_msgs.msg.Pose, self.set_goal_cb)
        rospy.Subscriber("/dmp_executor/initial_pos", geometry_msgs.msg.Pose, self.set_initial_pos_cb)
        rospy.Subscriber("/dmp_executor/dmp_name", geometry_msgs.msg.Pose, self.set_initial_pos_cb)
        '''
        self.tf_listener = tf.TransformListener()
        self.cartesian_velocity_command_pub = "/arm_1/arm_controller/cartesian_velocity_command"
        self.number_of_sampling_points = 30
        self.goal_tolerance = 0.02
        self.vel_publisher = rospy.Publisher(self.cartesian_velocity_command_pub, TwistStamped, queue_size=1)
        self.feedforward_gain = 0.8
        self.feedback_gain = 6

        rospy.Subscriber("/dmp_executor/update_goal", Pose, self.update_goal_cb)
        self.path_pub = rospy.Publisher("/dmp_executor/debug_path", Path, queue_size=1)
        self.event_in = None
        self.goal = None
        self.initial_pos = None
        self.dmp_name = dmp_name
        self.tau = tau
        self.roll = roll_dmp.roll_dmp(self.dmp_name)


        # wait for MoveIt! to come up
        move_group = "move_group"
        self.group_name = "arm_1"

        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.commander = moveit_commander.RobotCommander()
        self.state = moveit_commander.RobotState()
        self.joint_names = self.commander.get_joint_names(self.group_name)
        self.link_names = self.commander.get_link_names(self.group_name)

        self.kinematics = kinematics.Kinematics("arm_1")

        i = raw_input("enter to start")


        self.goal_change_point = np.array([0.4893742648357876, -0.19447904947512895, 0.11855376189875282])



    def move_arm(self, target_pose):

        print target_pose
        self.group.set_joint_value_target(target_pose)
        self.group.go()
       
        #self.group.execute(plan)

    def event_in_cb(self, msg):

        self.event_in = msg.data

    def update_goal_cb(self, msg):

        self.goal = np.array([msg.position.x, msg.position.y, msg.position.z, 0.0, 0.0, 0.0])
        print "Goal updated", self.goal
        self.roll.update_goal(self.goal)

    def set_initial_pos_cb(self, msg):

        self.initial_pos = msg.data
            
    def generate_trajectory(self, goal, initial_pos):

        self.goal = np.array([goal[0], goal[1], goal[2], 0.0, 0.0, 0.0])
        self.initial_pos = np.array([initial_pos[0], initial_pos[1], initial_pos[2], 0.0, 0.0, 0.0])
        self.pos, self.vel, self.acc = self.roll.roll(self.goal,self.initial_pos, self.tau)
        self.roll.reset_state()
    
    def publish_path(self):

        path = Path()
        path.header.frame_id = "/base_link"
        for itr in range(self.pos.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = self.pos[itr,0]
            pose_stamped.pose.position.y = self.pos[itr,1]
            pose_stamped.pose.position.z = self.pos[itr,2]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)

    def generate_feedback(self, current_pos, expected_pos, feedback_gain):

        return (expected_pos - current_pos) * self.tau * feedback_gain




    def trajectory_controller(self, changed_goal):
        
        goal_is_changed = False
        count = 0
        
        pos = np.zeros((6)) 
        vel = pos 
        acc = vel
        planned_trajectory = []
        dt = 0.01
        while True:
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
        current_pos = np.array([trans[0], trans[1], trans[2]])
        distance = np.linalg.norm((self.goal[:3] - current_pos))
        followed_trajectory = []

        pos, vel, acc = self.roll.step(self.tau)
        while distance > self.goal_tolerance and not rospy.is_shutdown() :

            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/arm_link_5', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            


            message = TwistStamped()
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance = np.linalg.norm((self.goal[0:3] - current_pos))
            
            '''
            Goal modification
            '''
            if np.linalg.norm((current_pos - self.goal_change_point)) < 0.01 and goal_is_changed == False:
                goal_is_changed = True
                self.goal = np.array([changed_goal[0], changed_goal[1], changed_goal[2], 0.0, 0.0, 0.0])
                self.roll.update_goal(self.goal)

            #rolling dmp step by step
            if np.linalg.norm((pos[0:3] - current_pos)) < 0.005:    
                pos, vel, acc = self.roll.step(self.tau)
                followed_trajectory.append(current_pos)
                pos_ = pos.copy()
                pos_ = pos_[0:3]
                planned_trajectory.append(pos_.copy())


            vel_x = self.feedforward_gain * vel[0] + self.feedback_gain * (pos[0] - current_pos[0])
            vel_y = self.feedforward_gain * vel[1] + self.feedback_gain * (pos[1] - current_pos[1])
            vel_z = self.feedforward_gain * vel[2] + self.feedback_gain * (pos[2] - current_pos[2])


            
            message.header.seq = count
            message.header.frame_id = "/base_link"
            message.twist.linear.x = vel_x
            message.twist.linear.y = vel_y
            message.twist.linear.z = vel_z
            self.vel_publisher.publish(message)
            count += 1
            

        message = TwistStamped()
        message.header.seq = count
        message.header.frame_id = "/base_link"
        message.twist.linear.x = 0.0
        message.twist.linear.y = 0.0
        message.twist.linear.z = 0.0
        
        
        self.vel_publisher.publish(message)
        return np.array(followed_trajectory), np.array(planned_trajectory)

    def execute(self, initial_pos):

        original_goal = np.array([0.50514965309, 0.1029934215751,  0.1])
        '''
        goals = np.array([[0.50514965309, 0.1029934215751,  0.1],   
                    [0.480514965309, 0.1229934215751,  0.12],
                    [0.430514965309, 0.1529934215751,  0.16],
                    [0.510514965309, 0.2529934215751,  0.09],
                    [0.450514965309, 0.2029934215751,  0.09]])
        '''
        goals = np.array([#[0.50514965309, 0.1029934215751,  0.1],   
            #[0.480514965309, 0.1229934215751,  0.12],
            #[0.430514965309, 0.1529934215751,  0.16],
            [0.450514965309, 0.2329934215751,  0.09],
            [0.450514965309, 0.2029934215751,  0.09]])
        goal_count = 3
        for goal in goals:
            goal_count += 1            
            for ijk in range(5):
                self.generate_trajectory(original_goal, initial_pos)
                self.publish_path()

                # use moveit to move to the initial position  
                start_pose = geometry_msgs.msg.PoseStamped()   
                start_pose.header.frame_id = "base_link"
                start_pose.pose.position.x = self.pos[0, 0]
                start_pose.pose.position.y = self.pos[0, 1]
                start_pose.pose.position.z = self.pos[0, 2]
                start_pose.pose.orientation.w = 1.0
                start_pose.pose.orientation.x = 0.987783314898
                start_pose.pose.orientation.y = 0.155722342076
                start_pose.pose.orientation.z = 0.0057864431987
                start_pose.pose.orientation.w = 0.0010918158567
                joint_space_pose = self.kinematics.inverse_kinematics(start_pose)
                self.move_arm(joint_space_pose)


                rospy.sleep(2)
                #wait for  user to give green signal

   
                followed_trajectory, planned_trajectory = self.trajectory_controller(goal)
                data = {'executed_trajectory': np.asarray(followed_trajectory).tolist()}
                file = "../data/experiments/ogm/goal_" + str(goal_count) + "trial_" + str(ijk) + ".yaml"

                with open(file, "w") as f:
                    yaml.dump(data, f)

                data = {'planned_trajectory': np.asarray(planned_trajectory).tolist()}
                file = "../data/experiments/ogm/plan_" + str(goal_count) + "trial_" + str(ijk) + ".yaml"

                with open(file, "w") as f:
                    yaml.dump(data, f)
                rospy.sleep(1)
                print "Done ........................."


if __name__ == "__main__":

    rospy.init_node("dmp_test")
    dmp_name = "../data/weights/weights_s04.yaml"
    tau = 0.5
    
    

    # s01
    #goal = [0.6565163014988611, 0.09613193867279159, -0.029017892027807135]
    #initial_pos = [0.595455037327542, 0.157326582524496, -0.06882172522527247]
    
    # s03
    '''
    goal = [0.6787391173619448, -0.1779881027574822, 0.02783503274142035]
    initial_pos = [0.6571846669990477, 0.16402033525929882, -0.026525658799735174]
    '''
    
    # s06
    goals = np.array([[0.50514965309, 0.1029934215751,  0.1],
                    [0.480514965309, 0.1229934215751,  0.12],
                    [0.430514965309, 0.1529934215751,  0.16],
                    [0.510514965309, 0.2529934215751,  0.09],
                    [0.450514965309, 0.2329934215751,  0.07]])
    initial_pos = [0.454890328161, -0.234996709813, 0.06]

    '''
    goals = np.array([[0.50, 0.15,  0.15],
                    [0.480514965309, 0.1229934215751,  0.058],
                    [0.500514965309, 0.1529934215751,  0.065],
                    [0.450514965309, 0.2529934215751,  0.062],
                    [0.460514965309, 0.2329934215751,  0.063]])
    initial_pos = [0.50, -0.15, 0.05]
    '''
    goal_count = 0
    obj = dmp_executor(dmp_name, tau)
    obj.execute(initial_pos)
        

        