#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg

def addCollisionObject(planning_scene_interface):
    # publish a demo scene
    p = geometry_msgs.msg.PoseStamped()

    # Frame that the pose is specified relative to
    p.header.frame_id = 'panda_link0'

    # specifies origin/zero orientation (Quaternion)
    p.pose.orientation.w = 1.0

    # translation components
    p.pose.position.x = 0
    p.pose.position.y = 0.5
    p.pose.position.z = 0.2

    # request to add object to planning scene
    planning_scene_interface.add_box("obstacle", p, (0.4, 0.2, 0.4))
    

if __name__ == "__main__":
    #initialise moveit and node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_obstacle', anonymous=True)

    #get handle to planning scene API
    scene = moveit_commander.planning_scene_interface.PlanningSceneInterface()

    #IMPORTANT: you must call sleep after the previous line!
    rospy.sleep(2)

    #add collision object to scene
    addCollisionObject(scene)


