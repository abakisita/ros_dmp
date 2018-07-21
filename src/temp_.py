import rospy
import nav_msgs.msg
import actionlib
import move_base_msgs.msg
import actionlib_msgs.msg
from mcr_manipulation_utils_ros.kinematics import Kinematics
import moveit_commander

if __name__ == "__main__":

    group_name = "move_group"
    rospy.init_node('move_base_client_test')
    group = moveit_commander.MoveGroupCommander(group_name)
    commander = moveit_commander.RobotCommander()
    state = moveit_commander.RobotState()
    joint_names = commander.get_joint_names(group_name)
    link_names = commander.get_link_names(group_name)

    kinematics = Kinematics("arm")
    
    start_pose = geometry_msgs.msg.PoseStamped()   
    start_pose.header.frame_id = "odom"
    start_pose.pose.position.x = 0.371 
    start_pose.pose.position.y = -0.062
    start_pose.pose.position.z = 0.662
    start_pose.pose.orientation.x = -0.502
    start_pose.pose.orientation.y = 0.502
    start_pose.pose.orientation.z = -0.498
    start_pose.pose.orientation.w = -0.498


    
    joint_space_pose = self.kinematics.inverse_kinematics(start_pose)
    group.set_joint_value_target(joint_space_pose)
    group.go()