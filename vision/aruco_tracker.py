import cv2
import cv2.aruco as aruco
import numpy as np
import time
from struct import *
import sys
import random
import ast
from scipy.spatial.transform import Rotation   
import serial
import asyncio

marker_image = None
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) 

# Uncomment below code to generate marker .pngs
# for marker_id in range(2, 5):
#     marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, 200, None, 1)
#     cv2.imwrite(f"marker{marker_id}.png", marker_image)
# marker_image = cv2.aruco.generateImageMarker(dictionary, 1, 200, marker_image, 1)
# cv2.imwrite("marker1.png", marker_image)

# Open video capture
inputVideo = cv2.VideoCapture(0)

# camera parameters
cameraMatrix = np.array([[715.03132426, 0.0, 462.25626965],
 [0.0, 715.67733789, 265.2425816],
 [0.0,        0,              1]])
distCoeffs = np.array([[0.05515529], [-0.20330285], [-0.00108968], [-0.00468666], [0.25046973]])

# Set coordinate system
markerLength = 29.40
objPoints = np.array([[-markerLength/2, markerLength/2, 0],
                      [markerLength/2, markerLength/2, 0],
                      [markerLength/2, -markerLength/2, 0],
                      [-markerLength/2, -markerLength/2, 0]], dtype=np.float32)

# Create ArUco detector
detectorParams = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, detectorParams)

origins = {
    1:  np.array([-65,-240,637,1]),
    2:  np.array([-65,-240,637,1]),
    3:  np.array([-65,-240,637,1]),
    4:  np.array([-65,-240,637,1]),
    5:  np.array([-65,-240,637,1]),
}

# Starting serial communication
arduinoData = serial.Serial('COM7', 115200, timeout=0.5) # initializing port and speed | COM outgoing
arduinoData.reset_output_buffer()
arduinoData.reset_input_buffer()
time.sleep(5)

i = 0

if __name__ == "__main__":
    # Main loop
    while inputVideo.isOpened():
        ret, image = inputVideo.read()
        if not ret:
            break

        h, w = image.shape[:2]

        # Calculate new camera matrix
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w,h), 1, (w,h))

        # Undistort
        dst = cv2.undistort(image, cameraMatrix, distCoeffs, None, newCameraMatrix)

        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]    
        imageCopy = dst.copy()

        # Detect markers
        corners, ids, _ = detector.detectMarkers(dst)
        
        # If at least one marker detected
        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(imageCopy, corners, ids)
            
            nMarkers = len(corners)
            rvecs, tvecs = [], []

            # Calculate pose for each marker
            for i in range(nMarkers):
                marker_id = ids[i][0]

                x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
                
                x_center = x_sum * 0.25
                y_center = y_sum * 0.25
                
                # print("x: ", x_center)
                # print("y: ", y_center)
                
                _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], cameraMatrix, distCoeffs)
                rvecs.append(rvec)
                tvecs.append(tvec)
                
                cRt, _ = cv2.Rodrigues(rvec)
                
                cRt = Rotation.from_matrix(cRt)
                angles = cRt.as_euler("zyx", degrees=True)
                if angles[0] < 0:
                    angles[0] = angles[0] + 360.0
                print("angles: ", int(angles[0]))

            # Draw axis for each marker
            for i in range(nMarkers):
                cv2.drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10)
            
            # send x, y, and theta over BT
            try:
                arduinoData.write(pack('3h', int(x_center), int(y_center), int(angles[0])))
            except Exception as e:
                print(f"Error writing to Arduino: {e}")
                
        else:
            # if can't find aruco marker, send error code
            try:
                arduinoData.write(pack('3h', 1000, 2000, 3000)) 
            except Exception as e:
                print(f"Error writing to Arduino: {e}")
                
        # Show resulting image and close window
        cv2.imshow("out", imageCopy)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            time.sleep(1)
            for i in range(10):
                arduinoData.write(pack('3h', 69, 69, 69)) # port closed code
            break

    # Release video capture and close windows
    inputVideo.release()
    cv2.destroyAllWindows()
    
    time.sleep(1)
    # arduinoData.reset_output_buffer()
    arduinoData.write(pack('3h', 69, 69, 69)) # port closed code
    time.sleep(1)
    arduinoData.close()