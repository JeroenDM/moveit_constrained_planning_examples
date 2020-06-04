#!/usr/bin/env python
"""
Advertise a GetMotionPlan Servic that uses computeCartesianPath under the hood
and therefore can only handle Cartesian motion planning.
"""
from __future__ import print_function

import sys
import time
import rospy
import rospkg
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv

GROUP_NAME = "panda_arm"
# GROUP_NAME = "manipulator"


class CartesianPlanningServer:
    def __init__(self):
        self.mc = moveit_commander.RobotCommander()
        self.mg = moveit_commander.MoveGroupCommander(GROUP_NAME)

        self.server = rospy.Service(
            "cartesian_planning_server",
            moveit_msgs.srv.GetMotionPlan,
            lambda x: self.handle_request(x)
        )

        # default settings for Cartesian planning
        self.eef_step = 0.01
        self.jump_threshold = 0.0

    def handle_request(self, request):
        """
        Extract Cartesian waypoints from a MotionPlanRequest
        and use computeCartesianPath to find a solution.
        """
        rospy.loginfo("Received cartesian motion planning request.")

        req = request.motion_plan_request
        print(req)

        resp = moveit_msgs.msg.MotionPlanResponse()

        resp.group_name = req.group_name

        assert(len(req.reference_trajectories) == 1)
        assert(len(req.reference_trajectories[0].cartesian_trajectory) == 1)

        cartesian_trajectory = req.reference_trajectories[0].cartesian_trajectory[0]
        waypoints = []
        for ctp in cartesian_trajectory.points:
            waypoints.append(ctp.point.pose)

        start_time = time.time()
        (plan, fraction) = self.mg.compute_cartesian_path(
            waypoints,
            self.eef_step,
            self.jump_threshold)
        resp.planning_time = time.time() - start_time

        if fraction == 1.0:
            resp.error_code.val = moveit_msgs.msg.MoveItErrorCodes.SUCCESS
            resp.trajectory = plan
        else:
            resp.error_code.val = moveit_msgs.msg.MoveItErrorCodes.GOAL_IN_COLLISION

        return resp


if __name__ == '__main__':
    rospy.init_node('cart_planning_server', anonymous=True)
    server = CartesianPlanningServer()
    print("Ready receive planning requests.")
    rospy.spin()
