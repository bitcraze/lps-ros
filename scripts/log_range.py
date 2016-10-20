#!/usr/bin/env python
# Converts the Crayflie driver generic log of the ranging to a RangeArray msg
# The log data packet is assumed to contain:
# range0..5, status
import rospy
from bitcraze_lps_estimator.msg import RangeArray
from crazyflie_driver.msg import GenericLogData


def callback(data):
    ranging = RangeArray()

    ranging.ranges = data.values[:6]

    state = int(data.values[6])
    valid = [False] * 6
    for i in range(6):
        valid[i] = (state & (1 << i)) != 0
    ranging.valid = valid

    ranging_pub.publish(ranging)

if __name__ == "__main__":
    rospy.init_node('log_range')

    ranging_pub = rospy.Publisher("ranging", RangeArray, queue_size=10)

    rospy.Subscriber(rospy.get_namespace()+"log_ranges", GenericLogData, callback)

    rospy.spin()
