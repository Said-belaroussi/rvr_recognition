#!/usr/bin/env python3

from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

import sys
import rospy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
import rospkg
import cv2

def callback(data):
    msg = Detection2DArray()
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    detection_2d_list = []
    print(data, "lol")
    for i in data:
        detection_2d_msg = Detection2D()
        detection_2d_msg.header = header
        print(detection_2d_msg)
        bounding_box_2d = BoundingBox2D()
        pose_2d = Pose2D()
        pose_2d.x = i['x']
        pose_2d.y = i["y"]
        # print(pose_2d)
        bounding_box_2d.size_x = i["width"]
        bounding_box_2d.size_y = i["height"]
        detection_2d_msg.bbox = bounding_box_2d
        detection_2d_list.append(detection_2d_msg)

    msg.detections = detection_2d_list
    print(detection_2d_list)
    pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node("oak_detector", anonymous=True)
    pub = rospy.Publisher("oak", Detection2DArray, queue_size=1)

    # instantiating an object (rf) with the RoboflowOak module
    rf = RoboflowOak(model="rvr_detection_v2", confidence=0.05, overlap=0.5,
    version="1", api_key="g0WaLaIt5ndRvBhoVPOh", rgb=True,
    depth=False, device=None, blocking=True)
    # Running our model and displaying the video output with detections
    while True:
        t0 = time.time()
        # The rf.detect() function runs the model inference
        result, frame, raw_frame, depth = rf.detect()
        predictions = result["predictions"]

        #{
        #    predictions:
        #    [ {
        #        x: (middle),
        #        y:(middle),
        #        width:
        #        height:
        #        depth: ###->
        #        confidence:
        #        class:
        #        mask: {
        #    ]
        #}
        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from your OAK
        #depth - depth map for raw_frame, center-rectified to the center camera

        # timing: for benchmarking purposes
        t = time.time()-t0
        print("INFERENCE TIME IN MS ", 1/t)
        #print("PREDICTIONS ", [p.json() for p in predictions])
        callback([p.json() for p in predictions])
        # setting parameters for depth calculation
        # max_depth = np.amax(depth)
        # cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        cv2.imshow("frame", frame)

        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        if cv2.waitKey(1) == ord('q'):
            break


