## Interface

This document contains a summary of the things I learned about the existing MoveIt interfaces to specify constrained planning problems in general, and planning along under-constrained end-effector paths more specifically.

### GetMotionPlan vs GetCartesianPath

Case 2 and 3 (see readme) are closely related to Cartesian path following. The simplest interface in MoveIt to describe this type of problem is the `GetCartesianPath` service. This service allows you to specify a list of waypoints and optionally add path constraints. The two relevant fields are shown below:
```yaml
# A sequence of waypoints to be followed by the specified link, 
# while moving the specified group, such that the group moves only
# in a straight line between waypoints
geometry_msgs/Pose[] waypoints

# Specify additional constraints to be met by the Cartesian path
Constraints path_constraints
```

A more general format to specify planning problems is the `MotionPlanRequest`, used by all planning plugins in MoveIt. This is far more general and can also be used for point-to-point planning with path constraints. `MotionPlanRequest` has a data field called `reference_trajectories` that can be (mis)used to specify Cartesian trajectories that we want to follow. The relevant fields in a `MotionPlanRequest` are:
```yaml
# No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
Constraints path_constraints

# A set of trajectories that may be used as reference or initial trajectories for (typically optimization-based) planners
# These trajectories do not override start_state or goal_constraints
GenericTrajectory[] reference_trajectories
```
We have to go quite some layers deep into the `GenericTrajectory` message to specify the actual Cartesian path, as I confusingly described in [this issue](https://github.com/ros-planning/moveit_msgs/issues/79).

In `MotionPlanRequest`, there is another field related to constraints, but I'm not sure whether we need to use this.
```yaml
# The constraints the resulting trajectory must satisfy
TrajectoryConstraints trajectory_constraints
```

Besides `GetCartesianPath` and `MotionPlanRequest`, there is also the `MotionSequenceRequest`. This allows you to specify a list of `MotionPlanningRequest`. This is useful if we want to split up a Cartesian path in different primitive segments (line, circle arc, ...). This is how the[pilz_industrial_motion](https://github.com/PilzDE/pilz_industrial_motion) package uses this message.

Both `GetCartesianPath` and `MotionPlanRequest` support a start state field in the response. This can come in handy as some tasks only specify the end-effector pose at the start, not the joint positions. These can then be chosen by the planner. The planner then becomes a **Generator* in the terminology used by the [MoveIt Task Constructor](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html)

### Discrete points vs continuous constrained region
Some cases involve planning along under-constrained end-effector paths (case 2 and 3). The end-effector path is intuitively specified as a sequence of discrete poses. When the path is a line or circle segment, it is also easy to represent it analytically.

For planners like [descartes](https://github.com/ros-industrial-consortium/descartes), the discrete specification is the most natural. [TrajOpt](https://github.com/ros-planning/moveit/tree/master/moveit_planners/trajopt) also works with this approach. In both cases, the constraints are only imposed at these discrete control points, not in between.

For constrained planning in OMPL, the planner expects generic equality constraint `F(x)=0`. This means that the planner wants to sample any configuration along the path, not only the ones at the discrete points. This can be achieved by specifying the Cartesian path as a region in space. For example, a straight line segment could be described using a `shape_msgs/SolidPrimitive.msg` in the `moveit_msgs/PositionConstraint` that described a narrow box or cylinder along the path.

An alternative approach (similar to the current MoveIt OMPL interface) is to implement the sampling of valid states on the MoveIt side and not use the generic `F(x)=0` interface. The sampler can then choose one of the end-effector poses along the discrete path and use inverse kinematics to generate joint space samples.
