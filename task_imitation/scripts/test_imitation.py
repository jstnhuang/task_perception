#! /usr/bin/env python

# A simple utility for triggering task imitations.
# Before running: edit bag_path
# After running, you may want to save the pose of the objects for visualization purposes.

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, Quaternion
from task_perception_msgs.msg import ImitateDemoAction, ImitateDemoGoal
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo, Image
from dbot_ros_msgs.srv import InitInteractiveObjectPose, InitInteractiveObjectPoseRequest
import actionlib
import rosbag
import rospy


def inches_to_m(inches):
    return inches * 0.0254


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
        marker.color.r = 142/255.0
        marker.color.g = 15/255.0
        marker.color.b = 19/255.0
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
    stack = '/home/jstn/data/demonstrations/stack.bag'
    arrange_cheezit = '/home/jstn/data/demonstrations/arrange_cheezit.bag'
    bag_path = arrange_cheezit

    pose = Pose()
    start_poses = {
        move_two: [('pringles_1k.obj', Pose(
            position=Point(0.718, -0.36, 0.76),
            orientation=Quaternion(0, 0, 0, 1))), ('cheezit_1k.obj', Pose(
                position=Point(0.668, 0.322, 0.77),
                orientation=Quaternion(0, 0, 0, 1)))],
        stack: [('pringles_1k.obj', Pose(
            position=Point(0.67, -0.133, 0.76),
            orientation=Quaternion(0, 0, 0, 1))), ('cheezit_1k.obj', Pose(
                position=Point(0.485, 0.203, 0.76),
                orientation=Quaternion(0, 0, 0, 1)))],
        arrange_cheezit: [('cheezit_1k.obj', Pose(
            position=Point(0.718, -0.36, 0.76),
            orientation=Quaternion(0, 0, 0, 1))), ('cheezit_1k.obj', Pose(
                position=Point(0.668, 0.322, 0.77),
                orientation=Quaternion(0, 0, 0, 1)))]
    }

    info_pub = rospy.Publisher(
        'demonstration_image/camera_info', CameraInfo, queue_size=1, latch=True)
    rgb_pub = rospy.Publisher(
        'demonstration_image/rgb', Image, queue_size=1, latch=True)
    depth_pub = rospy.Publisher(
        'demonstration_image/depth', Image, queue_size=1, latch=True)
    bag = rosbag.Bag(bag_path)
    now = rospy.Time.now()
    rgb_published = False
    depth_published = False
    for topic, msg, t in bag.read_messages():
        if topic == 'rgb_image' and not rgb_published:
            msg.header.stamp = now
            rgb_pub.publish(msg)
            rgb_published = True
        if topic == 'depth_image' and not depth_published:
            msg.header.stamp = now
            depth_pub.publish(msg)
            depth_published = True
        if topic == 'camera_info':
            msg.header.stamp = now
            info_pub.publish(msg)
    bag.close()

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

    while client.get_state() != GoalStatus.PENDING:
        rospy.sleep(0.1)

    # Init start poses
    #init_object = rospy.ServiceProxy('init_interactive_object_pose', InitInteractiveObjectPose)
    #for mesh_name, pose in start_poses[bag_path]:
    #    init_req = InitInteractiveObjectPoseRequest()
    #    init_req.mesh_name = mesh_name
    #    init_req.pose = pose
    #    init_object.call(init_req)

    client.wait_for_result()


if __name__ == '__main__':
    main()
