
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
            if 



        rospy.Subscriber("/mcr_navigation/direct_base_controller/coordinator/event_out", std_msgs.msg.String, self.dbc_event_cb)
        self.dbc_pose_pub = rospy.Publisher("/mcr_navigation/direct_base_controller/input_pose", PoseStamped, queue_size=1)
        self.dbc_event_pub = rospy.Publisher("/mcr_navigation/direct_base_controller/coordinator/event_in", std_msgs.msg.String, queue_size=1)