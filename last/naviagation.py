#! /usr/bin/env python3
from reference import*
import numpy as np
import cv2
import rospy

class NavigationClass:
    def __init__(self):
        rospy.init_node("navigation")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.end_pub = rospy.Publisher("/action_flag",Float64, queue_size=1)
        self.goal_list = list()
        self.sequence = 0
        self.waypoint_1 = MoveBaseGoal()
        self.waypoint_1.target_pose.header.frame_id='map'
        self.waypoint_1.target_pose.pose.position.x = 14.281277349701359
        self.waypoint_1.target_pose.pose.position.y = -10.01545638472057
        self.waypoint_1.target_pose.pose.orientation.w = 0.9993246864886264
        self.waypoint_1.target_pose.pose.orientation.z = -0.036744672735087046
        self.goal_list.append(self.waypoint_1)

    def run(self):
        if self.client.get_state() != GoalStatus.ACTIVE:
            self.client.send_goal(self.goal_list[self.sequence])
            self.sequence += 1
    

if __name__ == '__main__':
    nc = NavigationClass()
    while not rospy.is_shutdown():
        nc.run()
        nc.client.wait_for_result()

        if nc.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached the goal!")
            break
    end_msg = 1
    nc.end_pub.publish(end_msg)
