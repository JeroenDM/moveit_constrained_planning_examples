#!/usr/bin/env python
"""
Planning case where the robot moves it's end-effector in between some collision objects.
Only the position of the Cartesian path is specified, the orientation is free to choose.
"""
from __future__ import print_function

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg

import rviz_tools_py.rviz_tools

from tf.transformations import quaternion_from_euler


GROUP_NAME = "manipulator"

# approxamite start state, I do not have the exact one yet because of problems with the planner
start_joint_positions = [0.21927106253635525, -0.9660666580558694,
                         2.07367363790321, 0.2440246629802121, -1.119464824301489, -0.10815469451557415]


def create_pose_msg(xyzijk):
    quat = quaternion_from_euler(math.radians(
        xyzijk[3]), math.radians(xyzijk[4]), math.radians(xyzijk[5]))
    p = geometry_msgs.msg.Pose()
    p.position.x = xyzijk[0]
    p.position.y = xyzijk[1]
    p.position.z = xyzijk[2]
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return p


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('execute_cart_planning_example', anonymous=True)

    rvt = rviz_tools_py.rviz_tools.RvizMarkers(
        "/world", "/visualization_marker")
    rospy.sleep(0.5)  # magic wait time to make things work
    rvt.deleteAllMarkers()

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

    print(robot.get_current_state())

    waypoints = []

    start = create_pose_msg([1.0, 0.2, 0.4, 0, 90, 0])
    stop = create_pose_msg([1.0, -0.2, 0.4, 0, 90, 0])

    waypoints.append(start)
    waypoints.append(stop)

    # waypoints.append(move_group.get_current_pose().pose)

    for i, pose in enumerate(waypoints):
        rvt.publishAxis(pose, 0.1, 0.01)
        rvt.publishText(pose, "Pose {}".format(i), "black", 0.05)

    move_group.set_joint_value_target(start_joint_positions)
    move_group.go()

    # move_group.set_pose_target(waypoints[0])
    # move_group.go()

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.1,
        0.0)

    # if fraction == 1.0:
    #     move_group.execute(plan)
