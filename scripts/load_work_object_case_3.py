#!/usr/bin/env python
"""
Load work objects for case 3.
The work object is defined in an urdf file at:
    benchmark_planning_setups/setup_1_support/work/kingpin.urdf

So there is some code here to parse the urdf and publish it.
"""
import sys
import copy
import rospy
import rospkg
import rosparam
import moveit_commander
import urdfpy

from geometry_msgs.msg import Vector3, Quaternion, Pose, PoseStamped


# The location of the urdf file inside the setup 1 support package
REL_WORK_PATH = "/urdf/work/"

# I moved the task a bit along the x-axis
# but this code was not flexible enough to change the position
# of the work object, so I hardcoded the x offset for now
X_OFFSET = 0.3


def numpy_to_pose(arr):
    """ Numpy 4x4 array to geometry_msg.Pose
    Code from: https://github.com/eric-wieser/ros_numpy
    TODO move this to some utility module if I have one.
    """
    from tf import transformations
    assert arr.shape == (4, 4)

    trans = transformations.translation_from_matrix(arr)
    quat = transformations.quaternion_from_matrix(arr)

    return Pose(position=Vector3(*trans), orientation=Quaternion(*quat))


def remove_all_objects(scene):
    """ Given a planning scene, remove all known objects. """
    for name in scene.get_known_object_names():
        scene.remove_world_object(name)


def parse_urdf_file(package_name, work_name):
    """ Convert urdf file (xml) to python dict.
    Using the urdfpy package for now.
    Using the xml package from the standard library could be
    easier to understand. We can change this in the future
    if it becomes a mess.
    """
    rospack = rospkg.RosPack()

    filepath = rospack.get_path(package_name)
    filepath += REL_WORK_PATH

    urdf = urdfpy.URDF.load(filepath + work_name + ".urdf")

    d = {"links": {}, "joints": {}}

    for link in urdf.links:
        if link.name == "world" or link.name == "work":
            continue
        else:
            d["links"][link.name] = parse_link(link, filepath)

    for joint in urdf.joints:
        p = PoseStamped()
        p.header.frame_id = joint.parent
        p.pose = numpy_to_pose(joint.origin)

        d["joints"][joint.name] = {
            "pose": p,
            "parent": joint.parent,
            "child": joint.child
        }
    return d


def parse_link(link, mesh_path):
    """ Assume a link has only a single collision object.
        Assume this collision object is a box.
        Assume the link named "world" has no collision objects.
    link: a urdfpy.urdf.Link object
    mesh_path: absolute path of the folder where we have to fine the stl files
    """
    assert len(link.collisions) == 1
    assert link.name != "world"
    assert link.name != "work"
    collision = link.collisions[0]
    if collision.geometry.box is not None:
        data = {"type": "box", "size": link.collisions[0].geometry.box.size}
    elif collision.geometry.mesh is not None:
        data = {
            "type": "mesh",
            "filename": mesh_path + collision.geometry.mesh.filename,
            "scale": collision.geometry.mesh.scale
        }
    else:
        raise Exception("No mesh of box collision geometry found.")

    return data


def publish_parsed_urdf(parsed_urdf, scene):
    """ Publish link geometry for every joint's child.

    TODO: there is an ugly hardcoded x offset for now.    
    """
    for name, joint in parsed_urdf["joints"].items():
        # get the child link data
        link = parsed_urdf["links"][joint["child"]]

        pose_stamped = copy.deepcopy(joint["pose"])
        pose_stamped.pose.position.x += X_OFFSET

        # publish the child links collision geometry
        if link["type"] == "box":
            scene.add_box(
                joint["child"],
                pose_stamped,
                link["size"]
            )
        else:
            scene.add_mesh(
                joint["child"],
                pose_stamped,
                link["filename"],
                link["scale"]
            )


if __name__ == "__main__":
    rospy.init_node("publish_work")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)  # wait for the above things to setup

    remove_all_objects(scene)

    work = parse_urdf_file("setup_1_support", "kingpin")
    publish_parsed_urdf(work, scene)
    print("Done!")
