from reference import*

class State:
    def __init__(self):
        rospy.init_node("State")
        self.my_position = PoseWithCovarianceStamped()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_CB)
        rospy.Subscriber("/obstacle_flag", Float64, self.obstacle_CB)
        self.action_pub = rospy.Publisher("/action_flag", Float64, queue_size=3)
        self.recog_pub = rospy.Publisher("/recog_flag", Float64, queue_size=3)
        self.position_flag = 0
        self.action_flag = 0
        self.recog_flag = 0
        self.obstacle = 0
        self.init_flag = 0

    def pose_CB(self, data):
        self.my_position.pose.pose.position.x = data.pose.pose.position.x
        self.my_position.pose.pose.position.y = data.pose.pose.position.y
        self.my_position.pose.pose.position.z = data.pose.pose.position.z
        self.my_position.pose.pose.orientation.x = data.pose.pose.orientation.x
        self.my_position.pose.pose.orientation.y = data.pose.pose.orientation.y
        self.my_position.pose.pose.orientation.z = data.pose.pose.orientation.z
        self.my_position.pose.pose.orientation.w = data.pose.pose.orientation.w
        # print(self.my_position)

    def obstacle_CB(self, data):
        self.obstacle = data.data

    def action_state_CB(self, data):
        self.action_state = data

    def position_Check(self):
        if position_Checking(self.my_position,mission2and3_position):
            self.position_flag = 2
            self.init_flag = 0
        elif position_Checking(self.my_position,mission4_position):
            self.position_flag = 3
            self.init_flag = 0 
        elif position_Checking(self.my_position,mission5_position):
            self.position_flag = 4
            self.init_flag = 0           
        elif position_Checking(self.my_position,left_turn_position):
            self.position_flag = 5
            self.init_flag = 0
        elif position_Checking(self.my_position,right_turn_position):
            self.position_flag = 6
            self.init_flag = 0
        elif position_Checking(self.my_position,go_straight_position1) or position_Checking(self.my_position,go_straight_position2):
            self.position_flag = 7
            self.init_flag = 0
        elif position_Checking(self.my_position,end_position):
            self.position_flag = 8
            self.init_flag = 0
        
#### Action flag ####
# 0 => Naviagation
# 1 => LKAS
# 2 => Lane_change
# 3 => RoundAbout
# 4 => Intersection       
# 5 => turn left
# 6 => turn right
# 7 => go straight 
# 8 => Stop

    def action_pub_func(self):
        if self.position_flag == 2 and self.obstacle == 0: # 2 => obstacle position (no obstacle)
            self.action_flag = 1 # 1 => LKAS
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 2 and self.obstacle == 1: # 2 => obstacle position (static)
            self.action_flag = 2 # 2 => Lane_change
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 2 and self.obstacle == 2: # 2 => obstacle position (dynamic)
            self.action_flag = 9 # 9 => wait obstacle
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 3 and self.init_flag == 0: # 3 => roundabout position
            self.action_flag = 3 # 3 => Roundabout
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 4 and self.init_flag == 0: # 4 => intersection position
            self.action_flag = 4 # 4 => Intersection
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 5 and self.init_flag == 0: # 5 => left turn position
            self.action_flag = 5 # 5 => turn left
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 6 and self.init_flag == 0: # 6 => right turn position
            self.action_flag = 6 # 6 => turn right
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 7 and self.init_flag == 0: # 7 => go straight position
            self.action_flag = 7 # 7 => go straight
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 8 and self.init_flag == 0: # 8 => stop position
            self.action_flag = 8 # 8 => Stop
            self.action_pub.publish(self.action_flag)

        elif self.position_flag == 1 and self.init_flag == 0: 
            self.action_flag = 1
            self.action_pub.publish(self.action_flag)

        self.init_flag = 1 # for publish action_flag once


    def recog_pub_func(self):
        if self.position_flag == 2:
            self.recog_flag = 1
        else:
            self.recog_flag = 0
        self.recog_pub.publish(self.recog_flag)
        
        

def position_Checking(my_position, goal_position):
    margin = 0.35
    flag = False
    if goal_position.pose.pose.position.x - margin < my_position.pose.pose.position.x < goal_position.pose.pose.position.x + margin: 
        if goal_position.pose.pose.position.y - margin < my_position.pose.pose.position.y < goal_position.pose.pose.position.y + margin:
            flag = True
    
    return flag


##########  AMCL position def  ########### 
mission2and3_position = PoseWithCovarianceStamped()
mission2and3_position.pose.pose.position.x = 18.51818721653665
mission2and3_position.pose.pose.position.y = -10.357516300252342

mission4_position = PoseWithCovarianceStamped()
mission4_position.pose.pose.position.x = 27.499277138118288
mission4_position.pose.pose.position.y = -2.2480968963382297

mission5_position = PoseWithCovarianceStamped()
mission5_position.pose.pose.position.x = 24.638262845788826
mission5_position.pose.pose.position.y = -4.447233037617107

left_turn_position = PoseWithCovarianceStamped()
left_turn_position.pose.pose.position.x = 29.739517542273
left_turn_position.pose.pose.position.y = 0.6003899873906491

right_turn_position = PoseWithCovarianceStamped()
right_turn_position.pose.pose.position.x = 22.50431322067299
right_turn_position.pose.pose.position.y = -7.179273692891177

go_straight_position1 = PoseWithCovarianceStamped()
go_straight_position1.pose.pose.position.x = 15.875660905600153
go_straight_position1.pose.pose.position.y = -10.138075486986212

go_straight_position2 = PoseWithCovarianceStamped()
go_straight_position2.pose.pose.position.x = 17.487622202089128   #15.927982240737618
go_straight_position2.pose.pose.position.y = -6.842635687383047    #-6.979997119324259   

end_position = PoseWithCovarianceStamped()
end_position.pose.pose.position.x = 21.588289124270972
end_position.pose.pose.position.y = 0.006273624878868325


#### Obstacle flag ####
# 0 => No Obstacle
# 1 => Static Obstacle
# 2 => Dynamic Obstacle

#### Position flag ####
# 2 => obstacle
# 3 => roundabout
# 4 => intersection
# 5 => left turn
# 6 => right turn
# 7 => go straight
# 8 => end line

#### Recog flag ####
# 0 => deactivate obstcle Recog
# 1 => activate obstacle Recog

#### Action flag ####
# 0 => Naviagation
# 1 => LKAS
# 2 => Lane_change
# 3 => RoundAbout
# 4 => Intersection       
# 5 => turn left
# 6 => turn right
# 7 => go straight 
# 8 => Stop


if __name__ == '__main__':
    st = State()
    try:
        while not rospy.is_shutdown():
            st.position_Check()
            rospy.loginfo_throttle(3, "position flag: %s", st.position_flag)
            st.action_pub_func()
            rospy.loginfo_throttle(3, "action flag: %s", st.action_flag)
            st.recog_pub_func()
            rospy.loginfo_throttle(3, "recog flag: %s", st.recog_flag)
            rospy.loginfo_throttle(3, "------------------------")

    except rospy.ROSInterruptException:
        pass
