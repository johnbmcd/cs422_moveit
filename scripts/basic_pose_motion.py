#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def basic_pose_motion(move_group):
    # To specify a pose we first create an empty Pose msg
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    
    # Set pose_goal as target pose for planner
    move_group.set_pose_target(pose_goal)
    
    # Plan and execute
    plan = move_group.go(wait=True)
    
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()

if __name__ == "__main__":
    #initialise moveit and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_basic_pose_motion', anonymous=True)

    #select planning group
    group_name = "panda_arm"
    move_group = moveit_commander.move_group.MoveGroupCommander(group_name)

    #plan and execute motion
    basic_pose_motion(move_group)

