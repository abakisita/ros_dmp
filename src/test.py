#!/usr/bin/python
import numpy as np
import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Twist
from nav_msgs.msg import Path
import tf
import actionlib
import actionlib_msgs
import moveit_msgs
import moveit_commander
import mcr_manipulation_utils_ros.kinematics as kinematics
import yaml
from os.path import join
import roll_dmp
import move_base_msgs.msg

class dmp_executor():

    def __init__(self, dmp_name, tau):
        """
        Initialization
        """
        self.tf_listener = tf.TransformListener()
        self.cartesian_velocity_command_pub = "/arm_1/arm_controller/cartesian_velocity_command"
        self.number_of_sampling_points = 30
        self.goal_tolerance = 0.03
        self.vel_publisher_arm = rospy.Publisher(self.cartesian_velocity_command_pub,
                                                 TwistStamped, queue_size=1)
        self.vel_publisher_base = rospy.Publisher('/cmd_vel_prio_low', 
                                                Twist, queue_size=1)
        self.feedforward_gain = 60
        self.feedback_gain = 10
        self.sigma_threshold_upper = 0.085
        self.sigma_threshold_lower = 0.01


        rospy.Subscriber("/mcr_navigation/direct_base_controller/coordinator/event_out", std_msgs.msg.String, self.dbc_event_cb)
        self.dbc_pose_pub = rospy.Publisher("/mcr_navigation/direct_base_controller/input_pose", PoseStamped, queue_size=1)
        self.dbc_event_pub = rospy.Publisher("/mcr_navigation/direct_base_controller/coordinator/event_in", std_msgs.msg.String, queue_size=1)
        rospy.Subscriber('/mir_manipulation/mcr_arm_cartesian_control/sigma_values',
                         std_msgs.msg.Float32MultiArray, self.sigma_values_cb)
        self.path_pub = rospy.Publisher("/dmp_executor/debug_path",
                                         Path, queue_size=1)
        self.event_in = None
        self.goal = None
        self.initial_pos = None
        self.dmp_name = dmp_name
        self.tau = tau
        

        # wait for MoveIt! to come up
        move_group = "move_group"
        self.group_name = "whole_body"

        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # moveit 
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("whole_body") # take control of the hsrb arm
        self.group.allow_replanning(True) # 5 attempts
        robot_state=robot.get_current_state()
        self.group.set_planning_time(10) #10 seconds for the planner
        self.group.set_goal_tolerance(0.005)

        self.min_sigma_value = None
        self.deploy_wbc = True  
        self.dbc_event = None

        # Move base server
        self.move_base_client = actionlib.SimpleActionClient('move_base/move', move_base_msgs.msg.MoveBaseAction)
        self.move_base_client.wait_for_server()
        print "found move_base server"

        rospy.loginfo('Going to start')

    def sigma_values_cb(self, msg):
        self.min_sigma_value = min(msg.data)

    def move_base(self):
        move_base_goal = move_base_msgs.msg.MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        print self.move_base_client.send_goal(move_base_goal)

    def move_arm(self, target_pose):

        self.group.set_named_target('pregrasp')
        self.group.go()

    def event_in_cb(self, msg):

        self.event_in = msg.data

    def set_goal_cb(self, msg):

        self.goal = msg.data

    def set_initial_pos_cb(self, msg):

        self.initial_pos = msg.data

    def dbc_event_cb(self, msg):

        self.dbc_event = msg.data

    def bring_back_start_pose(self):
        
        PoseStamped_ = PoseStamped()
        PoseStamped_.header.frame_id = "odom"
        self.dbc_pose_pub.publish(PoseStamped_)
        
        event_ = std_msgs.msg.String()
        event_.data = 'e_start'
        self.dbc_event_pub.publish(event_)
        while True:
            if self.dbc_event == 'e_success':
                break


    def generate_trajectory(self, goal, initial_pos):

        goal = np.array([goal[0], goal[1], goal[2], 0.0, 0.0, 0.0])
        initial_pos = np.array([initial_pos[0], initial_pos[1], initial_pos[2], 0.0, 0.0, 0.0])
        self.roll = roll_dmp.roll_dmp(self.dmp_name, n_bfs=150)
        self.pos, self.vel, self.acc = self.roll.roll(goal,initial_pos, self.tau)

    def tranform_pose(self, pose):

        #transform goals to odom frame
        pose_stamped_ = geometry_msgs.msg.PoseStamped()   
        pose_stamped_.header.frame_id = "base_link"
        pose_stamped_.pose.position.x = pose[0]
        pose_stamped_.pose.position.y = pose[1]
        pose_stamped_.pose.position.z = pose[2]
        pose_stamped_.pose.orientation.x = 0.987783314898
        pose_stamped_.pose.orientation.y = 0.155722342076
        pose_stamped_.pose.orientation.z = 0.0057864431987
        pose_stamped_.pose.orientation.w = 0.0010918158567

        while not rospy.is_shutdown():
            try:
                pose_ = self.tf_listener.transformPose('odom', pose_stamped_)
                break
            except:
                continue
        return np.array([pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z])


    def publish_path(self):

        path = Path()
        path.header.frame_id = "/odom"
        for itr in range(self.pos.shape[0]):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = self.pos[itr,0]
            pose_stamped.pose.position.y = self.pos[itr,1]
            pose_stamped.pose.position.z = self.pos[itr,2]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)


    def trajectory_controller(self):
        
        count = 0
        previous_index = 0
        path = self.pos[:, 0:3].T
        path_x = path[0,:]
        path_y = path[1,:]
        path_z = path[2,:]
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/hand_palm_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        current_pos = np.array([trans[0], trans[1], trans[2]])
        
        distance = np.linalg.norm((np.array(path[:,path.shape[1] - 1]) - current_pos))
        followed_trajectory = []
        print "final pos is ", path[:,path.shape[1] - 1]

        old_pos_index = 0
        while distance > self.goal_tolerance and not rospy.is_shutdown() :
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/hand_palm_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            current_pos = np.array([trans[0], trans[1], trans[2]])
            distance = np.linalg.norm((np.array(path[:,path.shape[1] - 1]) - current_pos))
            dist = []
            for i in range(path.shape[1]):
                dist.append(np.linalg.norm((path[:,i] - current_pos)))
            index =  np.argmin(dist)
            
            if old_pos_index != index:
                followed_trajectory.append(current_pos)
                old_pos_index = index
            
            if index < previous_index:
                index = previous_index
            else : 
                previous_index = index

            # Delete this block later
            if (index > path.shape[1] - 1):
                break

            if index == path.shape[1] - 1:
                ind = index
            else:
                ind = index + 1
            
            vel_x = self.feedforward_gain * (path_x[ind] - path_x[index]) + self.feedback_gain * (path_x[ind] - current_pos[0])
            vel_y = self.feedforward_gain * (path_y[ind] - path_y[index]) + self.feedback_gain * (path_y[ind] - current_pos[1])
            vel_z = self.feedforward_gain * (path_z[ind] - path_z[index]) + self.feedback_gain * (path_z[ind] - current_pos[2])

            vel_x_arm = vel_x
            vel_y_arm = vel_y
            vel_z_arm = vel_z

            vel_x_base = 0.0
            vel_y_base = 0.0
            vel_z_base = 0.0
            ratio = 1.0

            if self.min_sigma_value != None and self.min_sigma_value < self.sigma_threshold_upper and self.deploy_wbc:
                    
                    ratio = (self.min_sigma_value - self.sigma_threshold_lower)/(self.sigma_threshold_upper - self.sigma_threshold_lower)

                    vel_x_arm = vel_x * (ratio)
                    vel_y_arm = vel_y * (ratio)

                    vel_x_base = vel_x * (1 - ratio)
                    vel_y_base = vel_y * (1 - ratio)

                    # Publish base velocity inside the if consition
                    vector_ = Vector3Stamped()
                    vector_.header.seq = count
                    vector_.header.frame_id = "/odom"
                    vector_.vector.x = vel_x_base
                    vector_.vector.y = vel_y_base
                    vector_.vector.z = vel_z_base
                    
                    vector_ = self.tf_listener.transformVector3('base_link', vector_)

                    message_base = Twist()
                    message_base.linear.x = vector_.vector.x
                    message_base.linear.y = vector_.vector.y
                    message_base.linear.z = vector_.vector.z           
                    self.vel_publisher_base.publish(message_base)
                    
            
            message_arm = TwistStamped()
            message_arm.header.seq = count
            message_arm.header.frame_id = "/odom"
            message_arm.twist.linear.x = vel_x_arm
            message_arm.twist.linear.y = vel_y_arm
            message_arm.twist.linear.z = vel_z_arm
            self.vel_publisher_arm.publish(message_arm)
            count += 1

        # stop arm and base motion after converging 
        message_base = Twist()
        message_base.linear.x = 0.0
        message_base.linear.y = 0.0
        message_base.linear.z = 0.0

               
        message_arm = TwistStamped()
        message_arm.header.seq = count
        message_arm.header.frame_id = "/odom"
        message_arm.twist.linear.x = 0
        message_arm.twist.linear.y = 0
        message_arm.twist.linear.z = 0
        
        self.vel_publisher_arm.publish(message_arm)
        
        if self.deploy_wbc :
            self.vel_publisher_base.publish(message_base)

        return np.array(followed_trajectory), self.pos

    def execute(self, goal, initial_pos):
        
        
        self.generate_trajectory(goal, initial_pos)
        pos = []
        for i in range(self.pos.shape[0]):
            pos.append(self.tranform_pose(self.pos[i,0:3]))
        pos = np.array(pos)
        self.pos = pos
        self.publish_path()

        # transform pose to base link 
        start_pose = geometry_msgs.msg.PoseStamped()   
        start_pose.header.frame_id = "odom"
        start_pose.pose.position.x = self.pos[0, 0]
        start_pose.pose.position.y = self.pos[0, 1]
        start_pose.pose.position.z = self.pos[0, 2]

        while not rospy.is_shutdown():
            try:
                start_pose = self.tf_listener.transformPose('base_link', start_pose)
                break
            except:
                continue
        start_pose.pose.orientation.x = 0.529
        start_pose.pose.orientation.y = -0.475
        start_pose.pose.orientation.z = 0.467
        start_pose.pose.orientation.w = 0.525
        
        self.move_arm(start_pose)
        #i = raw_input("enter to execute motion")
        rospy.sleep(1.0)
        rospy.loginfo('Executing motion')

        followed_trajectory, planned_trajectory = self.trajectory_controller()
        rospy.sleep(1.0)
        self.bring_back_start_pose()
        return followed_trajectory, planned_trajectory

if __name__ == "__main__":

    rospy.init_node("dmp_test")
    #dmp_name = raw_input('Enter the path of a trajectory weight file: ')# "../data/weights/weights_s04.yaml"
    dmp_name = "../data/weights/weights_square.yaml"
    #experiment_data_path = "../data/experiments/07_07_square/"
    experiment_data_path = ""
    #experiment_data_path = raw_input('Enter the path of a directory where the experimental trajectories should be saved: ')
    number_of_trials = int(raw_input('Enter the number of desired trials: '))
    tau = 1
    
    '''
    # inverse parabola
    initial_pos = [0.6239002655985795, 0.14898291966686508, 0.03856489515421835]
    goals = np.array([[0.4778817991671843, -0.12152428195625502, 0.04268073411707274],
                    [0.5278817991671843, -0.12152428195625502, 0.04268073411707274],
                    [0.4778817991671843, -0.07152428195625502, 0.04268073411707274],
                    [0.4778817991671843, -0.12152428195625502, 0.07268073411707274],
                    [0.5078817991671843, -0.14152428195625502, 0.05268073411707274]])
    '''
    """
     # inverse parabola
    initial_pos = [0.5229895370550418, -0.05849142739483036, 0.04235724362216553]
    # goals = np.array([[0.5377165842826458, 0.16117097470639746, 0.04435152496637762],
    #                   [0.5077165842826458, 0.16117097470639746, 0.04435152496637762],
    #                   [0.5377165842826458, 0.13117097470639746, 0.04435152496637762],
    #                   [0.5077165842826458, 0.18117097470639746, 0.04435152496637762],
    #                   [0.5677165842826458, 0.14117097470639746, 0.04435152496637762]])
    goals = np.array([[0.5077165842826458, 0.18117097470639746, 0.04435152496637762],
                      [0.5677165842826458, 0.14117097470639746, 0.04435152496637762]])

    """

    # square
    '''
    initial_pos = [0.40035065642615446, 0.0, 0.04981048864687017]
    goals = np.array([[0.5500212945684006, 0.0, 0.14814434495844467],
                    [0.5000212945684006, 0.0, 0.17814434495844467],
                    [0.5000212945684006, 0.0, 0.16814434495844467],
                    [0.5000212945684006, 0.09982072917296857, 0.09814434495844467]])
    # s01
    #goal = [0.6565163014988611, 0.09613193867279159, -0.029017892027807135]
    #initial_pos = [0.595455037327542, 0.157326582524496, -0.06882172522527247]
    '''
    # s03
    '''
    goal = [0.6787391173619448, -0.1779881027574822, 0.02783503274142035]
    initial_pos = [0.6571846669990477, 0.16402033525929882, -0.026525658799735174]
    '''
    """ 
    # s06
    
    goals = np.array([[0.50514965309, 0.1029934215751,  0.1],
                    [0.480514965309, 0.1229934215751,  0.12],
                    [0.430514965309, 0.1529934215751,  0.16],
                    [0.510514965309, 0.2529934215751,  0.09],
                    [0.450514965309, 0.2329934215751,  0.07]])
    initial_pos = [0.454890328161, -0.234996709813, 0.06]
    """
    # line
    

    # parabola, 07.07    
    # goals = np.array([[0.49, 0.34,  0.06],
    #                   [0.44, 0.34,  0.06],
    #                   [0.49, 0.44, 0.06],
    #                   [0.54, 0.39, 0.06],
    #                   [0.64, 0.49, 0.06]])
    # initial_pos = [0.42, -0.19, 0.11]

    # rectangle, 07.07
    goals = np.array([[0.50, 0.4, 0.689],
                      [0.32, 0.37,  0.11],
                      [0.33, 0.47, 0.11],
                      [0.31, 0.32, 0.11],
                      [0.35, 0.52, 0.11]])
    initial_pos = [0.534, 0.078, 0.689]

    # s06
    """
    goals = np.array([[0.4716486275306709, 0.019262871925355593, 0.074801434683877235]])
    initial_pos = [0.4512339629582322, 0.22913135658686382, 0.0610403383353328]
    """
    """
    goals = np.array([[0.4616486275306709, 0.019262871925355593, 0.084801434683877235],
                    [0.4916486275306709, 0.019262871925355593, 0.084801434683877235],
                    [0.4616486275306709, -0.001262871925355593, 0.084801434683877235],
                    [0.4616486275306709, 0.019262871925355593, 0.119801434683877235],
                    [0.4816486275306709, 0.019262871925355593, 0.074801434683877235]])
    initial_pos = [0.4512339629582322, 0.22913135658686382, 0.0610403383353328]
    """
    #[0.6712339629582322, 0.027913135658686382, -0.0110403383353328]

    '''
    goals = np.array([[0.50, 0.15,  0.15],
                    [0.480514965309, 0.1229934215751,  0.058],
                    [0.500514965309, 0.1529934215751,  0.065],
                    [0.450514965309, 0.2529934215751,  0.062],
                    [0.460514965309, 0.2329934215751,  0.063]])
    initial_pos = [0.50, -0.15, 0.05]
    '''
    goal_count = 0
    for goal in goals:
        goal_count += 1
        trial_count = 0
        for i in range(number_of_trials):
            trial_count += 1
            rospy.loginfo('Goal #%d, trial #%d' % (goal_count, trial_count))
            obj = dmp_executor(dmp_name, tau)
            followed_trajectory, planned_trajectory = obj.execute(goal, initial_pos)
            data = {'executed_trajectory': np.asarray(followed_trajectory).tolist()}
            file_name = join(experiment_data_path + "goal_" + str(goal_count)+ "_trial_"+ str(trial_count) + ".yaml")
            with open(file_name, "w") as f:
                yaml.dump(data, f)

            data = {'planned_trajectory': np.asarray(planned_trajectory).tolist()}
            file_name = join(experiment_data_path + "plan_" + str(goal_count) +"_trial_"+ str(trial_count) + ".yaml")
            with open(file_name, "w") as f:
                yaml.dump(data, f)
            
        