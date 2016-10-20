#!/usr/bin/env python

# Behaves like the other estimators but receives the position estimate from the
# Crazyflie internal EKF instead of calculating the estimate

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

import tf

from crazyflie_driver.msg import GenericLogData

ps = PoseStamped()
ps.pose.orientation.w = 1
ps.pose.orientation.x = 0
ps.pose.orientation.y = 0
ps.pose.orientation.z = 0
ps.pose.position.x = 0
ps.pose.position.y = 0
ps.pose.position.z = 0

def callback_pos(data):
    ps.pose.position.x = data.values[0]
    ps.pose.position.y = data.values[1]
    ps.pose.position.z = data.values[2]
    ps.header.frame_id = "/world"
    ps.header.stamp = rospy.Time.now()

    pose_pub.publish(ps)

    br = tf.TransformBroadcaster()
    br.sendTransform((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
                     (ps.pose.orientation.x, ps.pose.orientation.y,
                         ps.pose.orientation.z, ps.pose.orientation.w),
                     rospy.Time.now(),
                     rospy.get_namespace() + "base_link",
                     "world")

    point = Point()
    point = ps.pose.position
    position_pub.publish(point)

def callback_qt(data):
    ps.pose.orientation.w = data.values[0]
    ps.pose.orientation.x = data.values[1]
    ps.pose.orientation.y = data.values[2]
    ps.pose.orientation.z = data.values[3]

if __name__ == "__main__":
    rospy.init_node('lps_ekf_bridge')

    pose_pub = rospy.Publisher(rospy.get_namespace() + "pose", PoseStamped, queue_size=10)
    position_pub = rospy.Publisher(rospy.get_namespace() + "position", Point, queue_size=10)

    rospy.Subscriber("log_kfpos", GenericLogData, callback_pos)
    rospy.Subscriber("log_kfqt", GenericLogData, callback_qt)

    rospy.spin()
