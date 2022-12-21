"""
This script detects markers using Aruco from the webcam and draw pose
"""

# Import required packages
import numpy as np
import cv2

cameraMatrix = np.array([[1.01301031e+03, 0.00000000e+00, 6.23731386e+02],
 [0.00000000e+00, 1.01096127e+03, 3.74389860e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],])
distCoeffs = np.array([[ 0.04380239, -0.29073878,  0.00202948,  0.00187076,  0.38318386]])

aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Create parameters to be used when detecting markers:
parameters = cv2.aruco.DetectorParameters_create()

# Create video capture object 'capture' to be used to capture frames from the first connected camera:
capture = cv2.VideoCapture(0)

while True:
    # Capture frame by frame from the video capture object 'capture':
    ret, frame = capture.read()

    # We convert the frame to grayscale:
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners beloning to each id# We call the function 'cv2.aruco.detectMarkers()'
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dictionary, parameters=parameters)

    # Draw detected markers:
    frame = cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids, borderColor=(0, 255, 0))

    #frame = cv2.aruco.drawDetectedMarkers(image=frame, corners=rejectedImgPoints, borderColor=(0, 0, 255))
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeffs)
    print(rvecs, tvecs)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release everything:
capture.release()
cv2.destroyAllWindows()