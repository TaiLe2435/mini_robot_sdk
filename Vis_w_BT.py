# imports
import cv2
import numpy as np
import time

import serial # lib for serial communication

ret = 0.3316338761548716
cameraMatrix = np.array([[715.03132426, 0.0, 462.25626965],
 [0.0, 715.67733789, 265.2425816],
 [0.0,        0,              1]])
distCoeff = np.array([[0.05515529], [-0.20330285], [-0.00108968], [-0.00468666], [0.25046973]])

# function looking for clicks
def clickEvent(event, x, y, flags, params):
    global flag, point
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        flag = True 
        # displaying the coordinates
        print(x, ' ', y)
        point = (x,y)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
        flag = True
        # displaying the coordinates
        print(x, ' ', y)
        point = (x,y)

def euclid(center1, center2):
    dist = np.sqrt((center1[0]-center2[0])**2+(center1[1]-center2[1])**2)
    dir = np.arctan2(center2[0]-center1[0], center2[1]-center1[1])
    
    return dist, dir

# capturing video
cap = cv2.VideoCapture(0) # change this value until you get the correct cam
# cap.set(cv2.CAP_PROP_FPS, 10)
flag = False
prevCircle = None
dist = lambda x1, y1, x2, y2: (x1-x2)**2+(y1-y2)**2 # mini function

# arduinoData = serial.Serial('COM7', 115200) # initializing port and speed
# arduinoData.reset_input_buffer()
i = 0

while True:
    _, img = cap.read()
    # resizing for faster processing
    img = cv2.resize(img, (320,240), interpolation = cv2.INTER_AREA)
    # (1920.0 1080.0) or (1280.0 720.0)
    # creating window for mouse input
    
    # Calculate new camera matrix
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeff, (320,240), 1, (320,240))

    # Undistort
    dst = cv2.undistort(img, cameraMatrix, distCoeff, None, newCameraMatrix)
    
    cv2.namedWindow("Robot Tracking")
    cv2.setMouseCallback('Robot Tracking', clickEvent)
    if flag == True:
        cv2.circle(dst, point, 10, (255, 0, 0), -1)

    # converting to HSV
    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
   
    # HSV range for color and defining mask
    # change these for respective colors
    # link for HSV stuff
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fi.stack.imgur.com%2FTSKh8.png&imgrefurl=https%3A%2F%2Fstackoverflow.com%2Fquestions%2F47483951%2Fhow-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image&tbnid=Jx2H1bjYvu6n_M&vet=12ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ..i&docid=d4AswGhN6lbYWM&w=720&h=355&q=hsv%20range&ved=2ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fanswers.opencv.org%2Fupfiles%2F15181560142151344.png&imgrefurl=https%3A%2F%2Fanswers.opencv.org%2Fquestion%2F184281%2Fhow-are-hsv-values-interpreted-in-python%2F&tbnid=mpa5ObAswr1QPM&vet=12ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ..i&docid=06ORPhgZpk_9yM&w=743&h=477&itg=1&q=hsv%20range&ved=2ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ
    green_lower = np.array([25, 52, 72])             # change upper and lower values for colors
    green_upper = np.array([83, 255, 255])
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    
    blue_lower = np.array([94, 80, 2])      # 94       # change upper and lower values for colors
    blue_upper = np.array([110, 255, 255])   # 120
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
   
    # tracking green color
    contours, hierarchy=cv2.findContours(green_mask,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_TC89_L1)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        maxArea = max(contours, key = cv2.contourArea)
       
        # only taking largest area
        if(area > 10):
            x, y, w, h = cv2.boundingRect(maxArea)
            # finding center of target
            xC = int(x + w/2)
            yC = int(y + h/2)
            green_center = (xC, yC)
            # print(center)
           
            # plotting bounding box and center
            dst = cv2.rectangle(img, (x, y),
                                    (x + w, y + h),
                                    (0, 255, 0), 2)
            cv2.circle(dst, green_center, 10, (0, 255, 0), -1)
            cv2.putText(dst, "Robot", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0))
 
    # tracking blue color
    contours, hierarchy=cv2.findContours(blue_mask,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_TC89_L1)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        maxArea = max(contours, key = cv2.contourArea)
       
        # only taking largest area
        if(area > 10):
            x, y, w, h = cv2.boundingRect(maxArea)
            # finding center of target
            xC = int(x + w/2)
            yC = int(y + h/2)
            blue_center = (xC, yC)
            # print(center)
           
            # plotting bounding box and center
            dst = cv2.rectangle(dst, (x, y),
                                    (x + w, y + h),
                                    (0, 255, 0), 2)
            cv2.circle(dst, blue_center, 10, (0, 255, 0), -1)
            cv2.putText(dst, "Target", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 0))
   
    cv2.imshow("Robot Tracking", dst)
   
    
    # cmd = str(i) + '\n'
    # # arduinoData.write(cmd.encode())
    # i += 1
    
    # if i > 200:
    #     i = 0
    
    # l, theta = euclid(green_center, blue_center)
    
    # if l <= 75:
    #     move_flag = 0
    #     # print("stop")
    # elif l > 75:
    #     move_flag = 1
    #     # print("move")
    # print("l: ", l, " theta: ", -theta * 180 / np.pi) # 75 pixels away threshold
    
    # theta = int(-theta * 180 / np.pi)
    # cmd = str(theta) + '\n'
    # arduinoData.write(cmd.encode())
    
    # 'Esc' key to exit video streaming
    key = cv2.waitKey(1)
    if key ==27:
        break
    
    time.sleep(0.1)  # gives time to robot
   
cap.release()
cv2.destroyAllWindows()

end = "9999\n"
time.sleep(1)
# arduinoData.write(end.encode())
# time.sleep(1)
# arduinoData.close()