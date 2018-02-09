#! /usr/bin/env python

# A simple utility for triggering task imitations.
# Before running: edit bag_path
# After running, you may want to save the pose of the objects for visualization purposes.

from geometry_msgs.msg import Point, Pose, Quaternion
from task_perception_msgs.msg import ImitateDemoAction, ImitateDemoGoal
from visualization_msgs.msg import Marker
import actionlib
import rospy


def table_marker():
    marker = Marker()
    marker.header.frame_id = 'base_footprint'
    marker.ns = 'table'
    marker.type = Marker.CUBE
    marker.pose.orientation.w = 1
    marker.pose.position.x = inches_to_m(36)
    marker.pose.position.y = 0
    marker.pose.position.z = inches_to_m(29) + inches_to_m(1) / 2
    marker.scale.x = inches_to_m(30)
    marker.scale.y = inches_to_m(48)
    marker.scale.z = inches_to_m(1)
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1
    return marker


def publish_start_poses(publisher, objs):
    for mesh_name, pose in objs:
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.ns = mesh_name
        marker.type = Marker.MESH_RESOURCE
        marker.pose = pose
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 1
        marker.color.g = 0.2
        marker.color.b = 0.4
        marker.color.a = 1
        marker.mesh_resource = 'package://object_meshes/object_models/{}'.format(
            mesh_name)
        publisher.publish(marker)


def main():
    rospy.init_node('test_imitation')

    pringles_1 = '/home/jstn/data/demonstrations/pringles_1.bag'
    cheezit_1 = '/home/jstn/data/demonstrations/cheezit_1.bag'
    cheezit_2 = '/home/jstn/data/demonstrations/cheezit_2.bag'
    move_two = '/home/jstn/data/demonstrations/move_two.bag'
    bag_path = move_two

    pose = Pose()
    start_poses = {
        move_two: [('pringles_1k.obj', Pose(
            position=Point(0.718, -0.36, 0.76),
            orientation=Quaternion(0, 0, 0, 1))), ('cheezit_1k.obj', Pose(
                position=Point(0.668, 0.322, 0.77),
                orientation=Quaternion(0, 0, 0, 1)))]
    }

    publisher = rospy.Publisher(
        'visualization_marker', Marker, queue_size=10, latch=True)
    for i in range(5):
        if rospy.is_shutdown() or publisher.get_num_connections() > 0:
            break
        rospy.logwarn_throttle(1, 'Waiting for Rviz...')
        rospy.sleep(1)
    publisher.publish(table_marker())
    publish_start_poses(publisher, start_poses[bag_path])

    client = actionlib.SimpleActionClient('imitate_demo', ImitateDemoAction)
    while not client.wait_for_server(rospy.Duration(1.0)):
        rospy.logwarn_throttle(1, 'Waiting for imitation action.')
    goal = ImitateDemoGoal()
    goal.bag_path = bag_path
    client.send_goal(goal)
    client.wait_for_result()


if __name__ == '__main__':
    main()
