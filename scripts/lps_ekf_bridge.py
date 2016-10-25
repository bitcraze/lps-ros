#!/usr/bin/env python

# Behaves like the other estimators but receives the position estimate from the
# Crazyflie internal EKF instead of calculating the estimate

import rospy
from geometry_msgs.msg import Point
from crazyflie_driver.srv import UpdateParams

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

    # Set anchor position according to the position setup in ROS
    rospy.wait_for_service('update_params')
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

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

    if rospy.has_param("anchorpos/enable"):
        rospy.set_param("anchorpos/enable", 1)
        update_params(["anchorpos/enable"])

    position_pub = rospy.Publisher("crazyflie_position", Point, queue_size=10)

    rospy.Subscriber("log_kfpos", GenericLogData, callback)

    rospy.spin()
