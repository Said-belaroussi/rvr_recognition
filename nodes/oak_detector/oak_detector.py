from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rvr_recognition.msg import PointArray
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
import rospkg
import cv2

def callback(data):
    msg = Detection2DArray
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    detection_2d_list = []
    for i in data:
        detection_2d_msg = Detection2D
        detection_2d_msg.header = header
        detection_2d_msg.BoundingBox2D.Pose2D.x = i["x"]
        detection_2d_msg.BoundingBox2D.Pose2D.x = i["y"]
        detection_2d_msg.BoundingBox2D.size_x = i["width"]
        detection_2d_msg.BoundingBox2D.size_y = i["height"]
        detection_2d_list.append(detection_2d_msg)
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
        print("PREDICTIONS ", [p.json() for p in predictions])
        callback(predictions)
        # setting parameters for depth calculation
        # max_depth = np.amax(depth)
        # cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        #cv2.imshow("frame", frame)

        # how to close the OAK inference window / stop inference: CTRL+q or CTRL+c
        if cv2.waitKey(1) == ord('q'):
            break


