import cv2
import cv2.aruco as aruco
import numpy as np
import time

marker_image = None
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) 
marker_image = cv2.aruco.generateImageMarker(dictionary, 1, 200, marker_image, 1)
cv2.imwrite("marker1.png", marker_image)

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

origin = np.array([-80,0,0,1])

# Main loop
while inputVideo.isOpened():
    ret, image = inputVideo.read()
    if not ret:
        break

    imageCopy = image.copy()

    # Detect markers
    corners, ids, _ = detector.detectMarkers(image)

    # If at least one marker detected
    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(imageCopy, corners, ids)

        nMarkers = len(corners)
        rvecs, tvecs = [], []

        # Calculate pose for each marker
        for i in range(nMarkers):
            _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], cameraMatrix, distCoeffs)
            # print(tvec)
            # time.sleep(0.5)
            rvecs.append(rvec)
            tvecs.append(tvec)
            
            R, _ = cv2.Rodrigues(rvec)
            
            T = np.eye(4,4)
            T[3,3] = 1
            T[:3, :3] = R
            T[:3, 3] = tvec.reshape(3) 
            # print(T)
            time.sleep(1)
            
            origin_marker = np.dot(T, origin)
            marker_position = origin_marker[:3]
            print("Marker Position with respect to Origin:", marker_position)

        # Draw axis for each marker
        for i in range(nMarkers):
            cv2.drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 10)

    # Show resulting image and close window
    cv2.imshow("out", imageCopy)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

# Release video capture and close windows
inputVideo.release()
cv2.destroyAllWindows()


