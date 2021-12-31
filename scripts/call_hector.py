#!/usr/bin/env python
import rospy
from hector_nav_msgs.srv import GetRobotTrajectory

class Services():
    def __init__(self):
        rospy.wait_for_service('get_exploration_path')
        self.get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)


rospy.wait_for_service('get_exploration_path')
get_exploration_path = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)

a = get_exploration_path()
print(a)