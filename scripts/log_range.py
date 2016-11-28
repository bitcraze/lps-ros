#!/usr/bin/env python
# Converts the Crayflie driver generic log of the ranging to a RangeArray msg
# The log data packet is assumed to contain:
# range0..5, status
import rospy
from bitcraze_lps_estimator.msg import RangeArray
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Vector3, Point

from anchorpos import get_anchors_pos

import math

anchorpos = []
last_pos = PoseStamped()


def pose_cb(data):
    global last_pos
    last_pos = data


def callback(data):
    # Update the range array
    ranging = RangeArray()

    ranging.ranges = data.data[:6]

    state = int(data.data[6])
    valid = [False] * 6
    for i in range(6):
        valid[i] = (state & (1 << i)) != 0
    ranging.valid = valid

    ranging_pub.publish(ranging)

    # Create the ranging lines
    marker_array = MarkerArray()
    for i in range(6):
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.id = i+40
        marker.type = Marker.LINE_STRIP
        marker.lifetime = rospy.Duration(1.0)
        marker.ns = rospy.get_namespace()
        marker.action = 0

        # Start line at marker
        p = Point()
        p.x = anchorpos[i][0]
        p.y = anchorpos[i][1]
        p.z = anchorpos[i][2]
        marker.points.append(p)
        global last_pos

        # Create normalized direction vector
        dir_vect = Vector3()
        dir_vect.x = last_pos.pose.position.x - anchorpos[i][0]
        dir_vect.y = last_pos.pose.position.y - anchorpos[i][1]
        dir_vect.z = last_pos.pose.position.z - anchorpos[i][2]
        mag = math.sqrt(dir_vect.x**2 + dir_vect.y**2 + dir_vect.z**2)
        dir_vect.x /= mag
        dir_vect.y /= mag
        dir_vect.z /= mag

        # Apply range for the anchor to the direction vector
        dist = Point()
        dist.x = ranging.ranges[i]*dir_vect.x + anchorpos[i][0]
        dist.y = ranging.ranges[i]*dir_vect.y + anchorpos[i][1]
        dist.z = ranging.ranges[i]*dir_vect.z + anchorpos[i][2]
        marker.scale.x = 0.005
        marker.color.r = rospy.get_param("red",   1)
        marker.color.g = rospy.get_param("green", 0)
        marker.color.b = rospy.get_param("blue",  0)
        marker.color.a = 0.5
        marker.points.append(dist)
        marker_array.markers.append(marker)

    range_paths.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node('log_range')

    ranging_pub = rospy.Publisher("ranging", RangeArray, queue_size=10)

    rospy.Subscriber("log_ranges",
                     Float32MultiArray, callback)

    anchor_dict = get_anchors_pos()
    for name in anchor_dict:
        anchorpos.append((anchor_dict[name][0],
                          anchor_dict[name][1],
                          anchor_dict[name][2]))

    rospy.Subscriber("pose", PoseStamped, pose_cb)

    range_paths = rospy.Publisher(rospy.get_namespace()+"range_paths",
                                  MarkerArray, queue_size=10)

    rospy.spin()
