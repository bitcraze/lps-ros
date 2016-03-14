#!/usr/bin/env python
"""This script approximates the position of the crazyflie using least-means-square (LMS)
"""

import rospy
from bitcraze_lps_estimator.msg import RangeArray
from geometry_msgs.msg import Point
import tf

from anchorpos import get_anchors_pos

import numpy as np
from scipy.stats import norm
from scipy.optimize import minimize
import math

bias = np.array([
    -0.295,
    -0.231,
    -0.241,
    -0.261,
    -0.187,
    -0.201
    ])

last_estimate = np.array([0,0,0])

def func(p, data):
    """Here we compute our cost function: 1/2 * \sum_i (||x_i - p|| - d_i)^2, i.e. the sum
    of the difference between anchors Crazyflie
    """
    result = 0
    for anchor in anchor_positions:
        if data.valid[anchor]:
            result = result + math.pow(np.linalg.norm(anchor_positions[anchor] - p) - (data.ranges[anchor] - bias[anchor]), 2)
    return result * 0.5

def func_der(p, data):
    """This is the derivative of our cost function.
    """
    result = np.array([0,0,0])
    for anchor in anchor_positions:
        if data.valid[anchor]:
            result = result + (p - anchor_positions[anchor]) * (1 - (data.ranges[anchor] - bias[anchor])/np.linalg.norm(anchor_positions[anchor] - p))
    return result


def callback(data):
    # optimize cost function, initialized with our last estimate
    global last_estimate
    res = minimize(func, last_estimate, args=(data), jac=func_der)
    estimate = res.x
    pt = Point()
    pt.x = estimate[0]
    pt.y = estimate[1]
    pt.z = estimate[2]

    position_pub.publish(pt)

    br = tf.TransformBroadcaster()
    br.sendTransform((pt.x, pt.y, pt.z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base_link",
                     "world")

    last_estimate = estimate

if __name__ == "__main__":
    rospy.init_node('dwm_lms')

    anchor_positions = get_anchors_pos()

    position_pub = rospy.Publisher("crazyflie_position", Point, queue_size=10)


    rospy.Subscriber("ranging", RangeArray, callback)

    rospy.spin()
