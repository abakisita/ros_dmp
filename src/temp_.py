import rospy
import nav_msgs.msg
import actionlib
import move_base_msgs.msg
import actionlib_msgs.msg
import move_base_msgs.msg

if __name__ == "__main__":
    
    # Move base server
    rospy.init_node("test_node")
    move_base_client = actionlib.SimpleActionClient('move_base/move', move_base_msgs.msg.MoveBaseAction)
    move_base_client.wait_for_server()
    print "found move_base server"
    move_base_goal = move_base_msgs.msg.MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = 'map'
    print move_base_goal
    print move_base_client.send_goal(move_base_goal)
    """
    - Translation: [0.007, -0.010, 0.000]
    - Rotation: in Quaternion [0.000, 0.000, 0.618, 0.786]
            in RPY (radian) [0.000, -0.000, 1.333]
            in RPY (degree) [0.000, -0.000, 76.348]
    """