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

# camera parameters
cameraMatrix = np.array([[715.03132426, 0.0, 462.25626965],
 [0.0, 715.67733789, 265.2425816],
 [0.0,        0,              1]])
distCoeffs = np.array([[0.05515529], [-0.20330285], [-0.00108968], [-0.00468666], [0.25046973]])

# Open video capture
inputVideo = cv2.VideoCapture(1, cv2.CAP_DSHOW)
height = int(inputVideo.get(cv2.CAP_PROP_FRAME_HEIGHT))
width = int(inputVideo.get(cv2.CAP_PROP_FRAME_WIDTH))

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

i = 0

# Code for drawing predetermined paths
# height: 473 width: 636
# straight line
straight_left = (50, 236) 
straight_right = (580, 236) 

# circle
circle_center = (318, 236) # left (418, 236)
radius = 187 #134 r= 10


# figure 8
t = np.linspace(0, 2 * np.pi, 400)
x = np.sin(t) * 250 + 318
y = (np.sin(t) * np.cos(t)) * 200 + 236
curve = np.column_stack((x.astype(np.int32), y.astype(np.int32)))

x_circle = circle_center[0] + (radius * np.cos(t))
y_circle = circle_center[1] + (radius * np.sin(t))

# Green color in BGR 
color = (0, 255, 0) 
thickness = 5

print("Which path trajectory would you like? a) straight b) circle c) loop")
traj = input()
print("Trajectory chosen: %s" %traj)

fourcc = cv2.VideoWriter_fourcc(*'MJPG') #*'mp4v' *'MJPG'
out = cv2.VideoWriter('temp.avi', fourcc, 30.0, (width, height), isColor=True)

# Starting serial communication
arduinoData = serial.Serial('COM7', 115200, timeout=0.5) # initializing port and speed | COM outgoing
arduinoData.reset_output_buffer()
arduinoData.reset_input_buffer()
print("starting in 5 seconds")
time.sleep(5)

ctr = 0

if __name__ == "__main__":
    # Main loop
    while inputVideo.isOpened():
        ret, image = inputVideo.read()
        if not ret:
            break
        image = cv2.resize(image, (width,height), interpolation = cv2.INTER_LINEAR)
        h, w = image.shape[:2]
        
        # Calculate new camera matrix
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w,h), 1, (w,h))

        # Undistort
        dst = cv2.undistort(image, cameraMatrix, distCoeffs, None, newCameraMatrix)

        x, y, w, h = roi
        print(w)
        print(h)
        # dst = dst[y:y+h, x:x+w]    // crashes recording
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
                
                # multiply by value to convert to real world coordinates | check if camera calib changes it first
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
                # print("angles: ", int(angles[0]))
            
            # Draw axis for each marker
            for i in range(nMarkers):
                cv2.drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10)        
            
            #__________OPEN_LOOP_CONTROL_STUFF______________#
            
            # # for straight line
            if ctr < 110: # should change to stop when it reaches point rather than ctr
                try:
                    ctr += 1
                    arduinoData.write(pack('3h', 200, 200, 0))
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")
            elif ctr >= 100:
                try:
                    arduinoData.write(pack('3h', 1000, 2000, 3000))
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")
           
            # for circle
            # try:
            #     arduinoData.write(pack('3h', 96, 199, 0)) #89, 326 for r=10
            #     ctr+=1
            # except Exception as e:
            #     print(f"Error writing to Arduino: {e}")
            
            # for figure 8
            # if ctr < 30:
            #     # send x, y, and theta over BT
            #     try:
            #         ctr += 1
            #         arduinoData.write(pack('3h', 200, 200, 0))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
            # elif ctr >= 30 and ctr < 140: 
            #     try:
            #         ctr += 1
            #         arduinoData.write(pack('3h', 59, 295, 0))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
            # elif ctr >= 140 and ctr < 185:
            #     try:
            #         ctr += 1
            #         arduinoData.write(pack('3h', 200, 200, 0))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
            # elif ctr >= 185 and ctr < 315:
            #     try:
            #         ctr += 1
            #         arduinoData.write(pack('3h', 372, 64, 0))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
            # elif ctr >= 315 and ctr < 330:
            #     try:
            #         ctr += 1
            #         arduinoData.write(pack('3h', 200, 200, 0))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
            # elif ctr >= 330:
            #     try:
            #         arduinoData.write(pack('3h', 1000, 2000, 3000))
            #     except Exception as e:
            #         print(f"Error writing to Arduino: {e}")
       
        else:
                # if can't find aruco marker, send error code
            try:
                arduinoData.write(pack('3h', 1000, 2000, 3000)) 
            except Exception as e:
                print(f"Error writing to Arduino: {e}")              
                
        # Show resulting image and close window
        if traj == "a":
            imageCopy = cv2.line(imageCopy, straight_left, straight_right, color, thickness) 
        elif traj == "b":
            imageCopy = cv2.circle(imageCopy, circle_center, radius, color, thickness)
        elif traj == "c":
            imageCopy = cv2.polylines(imageCopy, [curve], False, color, thickness)
        out.write(imageCopy)
        cv2.imshow("Tracking", imageCopy)
        # print(ctr)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            try:
                arduinoData.write(pack('3h', 1000, 2000, 3000)) 
            except Exception as e:
                print(f"Error writing to Arduino: {e}") 
            break

    # Release video capture and close windows
    inputVideo.release()
    out.release()
    arduinoData.close()
    cv2.destroyAllWindows()