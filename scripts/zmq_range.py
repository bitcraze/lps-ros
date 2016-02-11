#!/usr/bin/env python
# Bridge between ZMQ ranging packet that comes from the Python Crazyflie client
# to ROS RangeArray message
import rospy
from bitcraze_lps_estimator.msg import RangeArray

import zmq

if __name__ == "__main__":
    rospy.init_node('pos_zmq')

    ranging_pub = rospy.Publisher("ranging", RangeArray, queue_size=10)

    context = zmq.Context()
    ranging_socket = context.socket(zmq.PULL)
    ranging_socket.connect("tcp://127.0.0.1:7778")

    while not rospy.is_shutdown():
        ranging_json = ranging_socket.recv_json()

        ranging = RangeArray()
        ranging.ranges = ranging_json["ranges"]

        valid = [False] * 8
        for i in range(8):
            valid[i] = (ranging_json["state"] & (1 << i)) != 0

        ranging.valid = valid

        ranging_pub.publish(ranging)
