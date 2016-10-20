#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from anchorpos import get_anchors_pos


def init_3d_marker(pos):
    marker = Marker()

    marker.header.frame_id = "/world"
    marker.header.stamp = rospy.Time.now()

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    marker.lifetime = rospy.Duration()

    return marker


def publish_anchors(publisher, anchors_positions):
    array = MarkerArray()

    for name in anchors_positions:
        marker = init_3d_marker(anchors_positions[name])

        marker.id = name

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1

        marker.type = 1

        marker.lifetime = rospy.Duration()

        array.markers.append(marker)

    publisher.publish(array)


def publish_copter(publisher, pos):
    marker = init_3d_marker(pos)

    marker.id = 1000

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    marker.type = 1

    publisher.publish(marker)


def callback(data):
    publish_copter(tag_pub, [data.x, data.y, data.z])

if __name__ == "__main__":
    rospy.init_node("dwm_viz")

    anchors_positions = get_anchors_pos()

    anchors_pub = rospy.Publisher("anchors_markers", MarkerArray, queue_size=1,
                                  latch=True)
    tag_pub = rospy.Publisher(rospy.get_namespace()+"marker", Marker, queue_size=1,
                              latch=True)

    rospy.Subscriber(rospy.get_namespace()+"position", Point, callback)

    publish_anchors(anchors_pub, anchors_positions)
    publish_copter(tag_pub, [0, 0, 0])

    rospy.spin()
