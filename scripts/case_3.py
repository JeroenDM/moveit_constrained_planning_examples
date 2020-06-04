#!/usr/bin/env python
"""
Planning case 3, where a Kuka kr5 has a welding torch
and we have three simple straight lines it has to weld.
But the work object makes that we need some freedom around the torch axis
and also to option to push or drag the torch at he end of a weld segment
to avoid collision.
"""
from __future__ import print_function

import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg

import rviz_tools_py.rviz_tools

from tf.transformations import quaternion_from_euler

# GROUP_NAME = "panda_arm"
GROUP_NAME = "manipulator"

# test points to move along
# given as [x, y, x, rx, ry, rz]
example_points = [
    [0.99, -0.15, 0, -135, 0, 180],
    [0.71, -0.15, 0, -135, 0, 180],
    [0.71, -0.15, 0, 0, -135, 0],
    [0.71,  0.15, 0, 0, -135, 0],
    [0.71, 0.15, 0, 135, 0, 180],
    [0.99, 0.15, 0, 135, 0, 180],
]

start_position = [-0.05681047048790813, -0.6420693137556103, 1.9044120004085383,
                  5.4525997821297505, -0.8193171725866021, -5.064777642647267]


def create_pose_msg(xyzijk):
    quat = quaternion_from_euler(math.radians(
        pt[3]), math.radians(pt[4]), math.radians(pt[5]))
    p = geometry_msgs.msg.Pose()
    p.position.x = pt[0] + 0.3
    p.position.y = pt[1]
    p.position.z = pt[2]
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]
    return p


if __name__ == '__main__':
    rospy.init_node('execute_cart_planning_example', anonymous=True)

    # setup connection with rviz
    rvt = rviz_tools_py.rviz_tools.RvizMarkers("/world", "/visual_markers")
    rospy.sleep(0.5)  # magic wait time to make things work
    rvt.deleteAllMarkers()

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

    # print(robot.get_current_state())

    waypoints = []
    # waypoints.append(move_group.get_current_pose().pose)

    for pt in example_points:
        pose = create_pose_msg(pt)
        waypoints.append(pose)

    for i, pose in enumerate(waypoints):
        rvt.publishAxis(pose, 0.2, 0.02)
        rvt.publishText(pose, "Pose {}".format(i), "black", 0.05)

    # move_group.set_pose_target(waypoints[0])
    # move_group.go()
    move_group.set_joint_value_target(start_position)
    move_group.go()

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.1,
        0.0)

    if fraction == 1.0:
        move_group.execute(plan)
