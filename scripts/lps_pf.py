#!/usr/bin/env python
import rospy
from bitcraze_lps_estimator.msg import RangeArray
from geometry_msgs.msg import Point
import tf

from anchorpos import get_anchors_pos

import pfilter


def callback(data):
    nvalid = 0
    for anchor in anchor_positions:
        if data.valid[anchor] and data.ranges[anchor] > 0:
            pf.addRangeMeasurement(anchor, anchor_positions[anchor],
                                   data.ranges[anchor], 0.3)
        nvalid += 1

    if nvalid >= 4:
        pf.update()

    estimate = pf.getEstimate()
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

if __name__ == "__main__":
    rospy.init_node('dwm_pf')

    anchor_positions = get_anchors_pos()

    pf = pfilter.ParticleFilter(200, 0.1, (10, 10, 2))
    position_pub = rospy.Publisher("crazyflie_position", Point, queue_size=10)

    rospy.Subscriber("ranging", RangeArray, callback)

    rospy.spin()
