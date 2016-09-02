#!/usr/bin/env python

# Behaves like the other estimators but receives the position estimate from the
# Crazyflie internal EKF instead of calculating the estimate

import rospy
from geometry_msgs.msg import Point

import tf

from crazyflie_driver.msg import GenericLogData


def callback(data):
    pt = Point()
    pt.x = data.values[0]
    pt.y = data.values[1]
    pt.z = data.values[2]

    position_pub.publish(pt)

    br = tf.TransformBroadcaster()
    br.sendTransform((pt.x, pt.y, pt.z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     rospy.get_namespace() + "base_link",
                     "world")

if __name__ == "__main__":
    rospy.init_node('lps_ekf_bridge')

    position_pub = rospy.Publisher("crazyflie_position", Point, queue_size=10)

    rospy.Subscriber("log_kfpos", GenericLogData, callback)

    rospy.spin()
