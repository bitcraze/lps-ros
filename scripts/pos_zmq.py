#!/usr/bin/env python
# Bridge between a ROS point topic containing the Crazyflie position to ZMQ
# position messages for the Crazyflie python contoller
import rospy
from geometry_msgs.msg import Point

import zmq


def callback(data):
    pos_socket.send_json(
            {"pos": [data.x, data.y, data.z], "angle": 0, "detect": True})

if __name__ == "__main__":
    rospy.init_node('pos_zmq')

    rospy.Subscriber("crazyflie_position", Point, callback)

    context = zmq.Context()
    pos_socket = context.socket(zmq.PUSH)
    pos_socket.bind("tcp://127.0.0.1:7777")

    rospy.spin()
