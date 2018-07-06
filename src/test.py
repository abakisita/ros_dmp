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
        self.min_sigma_value = None
        self.deploy_wbc = True  
        rospy.loginfo('Going to start')

    def sigma_values_cb(self, msg):
        self.min_sigma_value = min(msg.data)

    def move_arm(self, target_pose):

        print target_pose
        self.group.set_joint_value_target(target_pose)
        self.group.go()
       
        #self.group.execute(plan)

    def event_in_cb(self, msg):

        self.event_in = msg.data

    def set_goal_cb(self, msg):

        self.goal = msg.data

    def set_initial_pos_cb(self, msg):

        self.initial_pos = msg.data


    def generate_trajectory(self, goal, initial_pos):

        goal = np.array([goal[0], goal[1], goal[2], 0.0, 0.0, 0.0])
        initial_pos = np.array([initial_pos[0], initial_pos[1], initial_pos[2], 0.0, 0.0, 0.0])
        self.roll = roll_dmp.roll_dmp(self.dmp_name)
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
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/arm_link_5', rospy.Time(0))
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
                (trans,rot) = self.tf_listener.lookupTransform('/odom', '/arm_link_5', rospy.Time(0))
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
        start_pose.pose.orientation.x = 0.987783314898
        start_pose.pose.orientation.y = 0.155722342076
        start_pose.pose.orientation.z = 0.0057864431987
        start_pose.pose.orientation.w = 0.0010918158567


        
        joint_space_pose = self.kinematics.inverse_kinematics(start_pose)
        self.move_arm(joint_space_pose)
        # i = raw_input("enter to execute motion")
        rospy.sleep(1.0)
        rospy.loginfo('Executing motion')

        return self.trajectory_controller()

if __name__ == "__main__":

    rospy.init_node("dmp_test")
    #dmp_name = raw_input('Enter the path of a trajectory weight file: ')# "../data/weights/weights_s04.yaml"
    dmp_name = "../data/weights/weights_line.yaml"
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
    
    
    goals = np.array([[0.5, 0.50, 0.18],
                    [0.5, 0.45, 0.18],
                    [0.45, 0.40, 0.18],
                    [0.5, 0.38, 0.18],
                    [0.48, 0.35, 0.18],])
    initial_pos = [0.42, -0.19, 0.18]
    

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
            
            k = raw_input('enter to start next run')

        k = raw_input('press enter to continue')