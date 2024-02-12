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
import matplotlib.pyplot as plt


def euclid(center1, center2):
    dist = np.sqrt((center1[0]-center2[0])**2 + (center1[1]-center2[1])**2)
    dir = np.arctan2(center2[0]-center1[0], center2[1]-center1[1]) * (180/np.pi)
    if dir < 0:
        dir = dir + 180.0
    return dist, dir

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

# Code for drawing predetermined paths
# height: 473 width: 636
# straight line
straight_left = (50, 236) 
straight_right = (580, 236) 

# circle
circle_center = (318, 236) # left (418, 236)
radius = 187 #134 r= 10


# figure 8
t = np.linspace(0, 2 * np.pi, 100)
x = np.sin(t) * 250 + 318
y = (np.sin(t) * np.cos(t)) * 200 + 236
curve = np.column_stack((x.astype(np.int32), y.astype(np.int32)))

x_circle = circle_center[0] + (radius * np.cos(t))
y_circle = circle_center[1] + (radius * np.sin(t))

# print(x_circle)
# print(y_circle)

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
m = 0

targetTheta = []

for j in range(48):
    desired_prev = [x_circle[j], y_circle[j]]    # heading is 180 for 0 and 1
    desired_current = [x_circle[j+1], y_circle[j+1]] # heading is 135 for 50 and 51
    
    _ , theta = euclid(desired_prev, desired_current)
    # l, _ = euclid(current, desired_current)
    targetTheta.append(theta)
    
for k in range(50): # deals with wrapping
        desired_prev = [x_circle[49+k], y_circle[49+k]]    # heading is 180 for 0 and 1
        desired_current = [x_circle[49+k+1], y_circle[49+k+1]] # heading is 135 for 50 and 51
        
        _ , theta = euclid(desired_prev, desired_current)
        # l, _ = euclid(current, desired_current)
        targetTheta.append(theta + 180)
# print(targetTheta)
print(targetTheta[0])
# plt.plot(targetTheta)
# plt.show()

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
             
            if traj == "a": # tune controller, add D controller part
                kP = 0.75
                line_error = (straight_left[1] - y_center) # goal path
                pos_error = (straight_right[0] - x_center) # goal position
                
                pos_error = kP * pos_error
                line_error = 4.5 * line_error
                
                # print("pos error: %i" %pos_error)
                # print("line error: %i" %line_error)
                
                velocity_left = pos_error + line_error;
                if velocity_left > 400:
                    velocity_left = 400
                elif velocity_left < -400:
                    velocity_left = -400
                    
                velocity_right = pos_error - line_error;
                if velocity_right > 400:
                    velocity_right = 400
                elif velocity_right < -400:
                    velocity_right = -400
                    
                try:
                    arduinoData.write(pack('3h', int(velocity_left), int(velocity_right), 0)) 
                    # arduinoData.write(pack('3h', 1000, 2000, 3000)) 
                    print("left v: %i" %velocity_left)
                    print("right v: %i" %velocity_right)
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")  
                    
            if traj == "b":
                kP = 1.6
                kV = 1.2
                kC = 0.1
                
                current = [x_center, y_center]
                desired_prev = [x_circle[m], y_circle[m]]    # heading is 180 for 0 and 1
                desired_current = [x_circle[m+1], y_circle[m+1]] # heading is 135 for 50 and 51
                
                # _ , theta = euclid(desired_prev, desired_current)
                # l, _ = euclid(current, desired_current)
                r, _ = euclid(current, circle_center)
                # print("robot states: %i, %i, %i" %(x_center, y_center, angles[0]))
                # # print("desired states: %i, %i" %(x_circle[1], y_circle[1]))
                
                
                # if l < 10 and m < len(targetTheta):
                #     m = m + 1
                #     # print("increasing i: %i" %m)
                # elif m == len(targetTheta):
                #     print("end of trajectory reached")
                    
                # heading_error = targetTheta[0] - angles[0]
                heading_error = 0
                r_error = r - radius
                # pos_error = l
                pos_error = 0
               
                print("euclid stuff: %i, %i" %(r_error, int(targetTheta[m])))
               
                # handle angle wrapping for control
                if heading_error > 180:
                    heading_error = heading_error - 360
                elif heading_error < -180:
                    heading_error = heading_error + 360
               
                # controller
                velocity_right = 199 + kP * heading_error + kV * pos_error + kC * r_error
                velocity_left = 96 - kP * heading_error - kV * pos_error - kC * r_error

                # set max and min values
                if velocity_left > 400:
                    velocity_left = 400
                elif velocity_left < -400:
                    velocity_left = -400

                if velocity_right > 400:
                    velocity_right = 400
                elif velocity_right < -400:
                    velocity_right = -400
                    
                print("vL: %i" %(velocity_left))
                print("vR: %i" %(velocity_right))
                
                try:
                    # arduinoData.write(pack('3h', int(velocity_left), int(velocity_right), 0)) 
                    arduinoData.write(pack('3h', int(velocity_left), int(velocity_right), 0)) 
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")     
                
                
            if traj == "c":
                kP = 0.5
            
            # Draw axis for each marker
            for i in range(nMarkers):
                cv2.drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10)        
            
       
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