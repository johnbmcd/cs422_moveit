#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from math import pi

def basic_motion(move_group):
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

if __name__ == "__main__":
    #initialise moveit and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_basic_motion', anonymous=True)

    #select planning group
    group_name = "panda_arm"
    move_group = moveit_commander.move_group.MoveGroupCommander(group_name)

    #plan and execute motionp
    basic_motion(move_group)
