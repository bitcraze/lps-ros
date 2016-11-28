#!/usr/bin/env python

# Behaves like the other estimators but receives the position estimate from the
# Crazyflie internal EKF instead of calculating the estimate

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from crazyflie_driver.srv import UpdateParams
from std_msgs.msg import Float32MultiArray

import tf

ps = PoseStamped()
ps.pose.orientation.w = 1
ps.pose.orientation.x = 0
ps.pose.orientation.y = 0
ps.pose.orientation.z = 0
ps.pose.position.x = 0
ps.pose.position.y = 0
ps.pose.position.z = 0


def callback_pos(data):
    ps.pose.position.x = data.data[0]
    ps.pose.position.y = data.data[1]
    ps.pose.position.z = data.data[2]
    ps.header.frame_id = "/world"
    ps.header.stamp = rospy.Time.now()

    pose_pub.publish(ps)

    br = tf.TransformBroadcaster()
    br.sendTransform(
            (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
            (ps.pose.orientation.x, ps.pose.orientation.y,
                ps.pose.orientation.z, ps.pose.orientation.w),
            rospy.Time.now(),
            rospy.get_namespace() + "base_link",
            "world")

    point = Point()
    point = ps.pose.position
    position_pub.publish(point)


def callback_qt(data):
    ps.pose.orientation.w = data.data[0]
    ps.pose.orientation.x = data.data[1]
    ps.pose.orientation.y = data.data[2]
    ps.pose.orientation.z = data.data[3]

if __name__ == "__main__":
    rospy.init_node('lps_ekf_bridge')

    pose_pub = rospy.Publisher(rospy.get_namespace() + "pose",
                               PoseStamped, queue_size=10)
    position_pub = rospy.Publisher(rospy.get_namespace() + "position",
                                   Point, queue_size=10)

    # Set anchor position according to the position setup in ROS
    rospy.wait_for_service('update_params')
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    enabled = rospy.get_param("anchorpos/enable")
    if not enabled:
        rospy.loginfo("Setting anchor position ...")
        n_anchors = rospy.get_param("n_anchors")
        for i in range(n_anchors):
            position = rospy.get_param("anchor{}_pos".format(i))
            rospy.loginfo("Anchor {} at {}".format(i, position))
            name = "anchorpos/anchor{}".format(i)
            rospy.set_param(name + "x", position[0])
            rospy.set_param(name + "y", position[1])
            rospy.set_param(name + "z", position[2])
            update_params([name + 'x', name + 'y', name + 'z'])

    rospy.Subscriber("log_kfpos", Float32MultiArray, callback_pos)
    rospy.Subscriber("log_kfqt", Float32MultiArray, callback_qt)
    if rospy.has_param("anchorpos/enable"):
        rospy.set_param("anchorpos/enable", 1)
        update_params(["anchorpos/enable"])

    position_pub = rospy.Publisher("crazyflie_position", Point, queue_size=10)

    rospy.spin()
