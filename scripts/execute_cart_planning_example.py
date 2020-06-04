#!/usr/bin/env python
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

GROUP_NAME = "panda_arm"
# GROUP_NAME = "manipulator"

def create_path(current_end_effector_pose):
    """
    This is the default Cartesian path used in many tutorials.
    """
    pts = []
    scale = 1.0
    wpose = current_end_effector_pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    pts.append(copy.deepcopy(wpose))
    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    pts.append(copy.deepcopy(wpose))
    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    pts.append(copy.deepcopy(wpose))
    return pts


def create_path_constraints(ee_link_name):
    """
    TODO add constraints to the planning message,
    now they are read from the ros parameters server
    for the descartes planner.
    """
    rospy.loginfo("Adding constraints for link {}".format(ee_link_name))
    con = moveit_msgs.msg.Constraints()
    con.name = "z_axis_free_constraint"
    ori_con = moveit_msgs.msg.OrientationConstraint()
    ori_con.absolute_x_axis_tolerance = 0
    return con


if __name__ == '__main__':
    rospy.init_node('execute_cart_planning_example', anonymous=True)

    # setup connection with rviz
    rvt = rviz_tools_py.rviz_tools.RvizMarkers("/panda_link0", "/visualization_marker")
    rospy.sleep(0.5)  # magic wait time to make things work
    rvt.deleteAllMarkers()

    rospy.wait_for_service("cartesian_planning_server", timeout=5.0)

    planning_service = rospy.ServiceProxy(
        "cartesian_planning_server", moveit_msgs.srv.GetMotionPlan)

    robot = moveit_commander.RobotCommander()
    move_group = moveit_commander.MoveGroupCommander(GROUP_NAME)

    # move the robot to a start state, so the demo always starts at
    # the same pose, then create path.
    move_group.set_named_target("ready")
    move_group.go()

    waypoints = create_path(move_group.get_current_pose().pose)

    for i, pose in enumerate(waypoints):
        rvt.publishAxis(pose, 0.1, 0.01)
        rvt.publishText(pose, "Pose {}".format(i), "black", 0.05)

    request = moveit_msgs.msg.MotionPlanRequest()
    generic_trajectory = moveit_msgs.msg.GenericTrajectory()
    cartesian_trajectory = moveit_msgs.msg.CartesianTrajectory()

    for waypoint in waypoints:
        cart_point = moveit_msgs.msg.CartesianTrajectoryPoint()
        cart_point.point.pose = waypoint
        cartesian_trajectory.points.append(cart_point)

    generic_trajectory.cartesian_trajectory.append(cartesian_trajectory)
    request.reference_trajectories.append(generic_trajectory)

    request.group_name = GROUP_NAME

    request.path_constraints = create_path_constraints(
        move_group.get_end_effector_link())

    response = planning_service(request)

    # print(response)

    if response.motion_plan_response.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        move_group.execute(response.motion_plan_response.trajectory)
    
