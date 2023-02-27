#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rvr_recognition.msg import PointArray
import rospkg
import cv2



def callback(data):
    np_arr = np.frombuffer(data.data, np.uint8)
    cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    positions = get_positions(cv2_img)
    msg = PointArray()
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    msg.points = positions
    pub.publish(msg)


def listener():
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(INPUT, CompressedImage, callback, queue_size=1, buff_size=2 ** 24)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 4:
        print("Error, wrong number of args")
        sys.exit(1)
    INPUT = args[1]
    OUTPUT = args[2]
    CAMERA = args[3]
    DETECTOR = init_detector(4, False)

    pub = rospy.Publisher(OUTPUT, PointArray, queue_size=1)

    NODE_NAME = "nn_detector"
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
