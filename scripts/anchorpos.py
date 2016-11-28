# Helper function to load the list of anchors from the ROS parameter server
import rospy


def get_anchors_pos():
    n = rospy.get_param("/n_anchors")
    anchors_pos = {}

    for i in range(n):
        anchors_pos[i] = rospy.get_param("/anchor{}_pos".format(i))

    return anchors_pos
