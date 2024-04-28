# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L1_motor as lm
import L1_lidar as lidar
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import L2_vector as lv
from time import sleep
from math import radians, pi

FOV = 1
EDGE_MARGIN = 5

_kernel = np.ones((5,5),np.uint8)

cap = cv2.VideoCapture(0)
print("Video capture:", cap)

Tracker = cv2.TrackerMIL_create()

def restartTracking(img):
    cv2.putText(img, "LOST", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    # ROI = cv2.selectROI("Tracking...", img, False)
    ROI = getInitialBounds(img)
    if ROI is None:
        return
    Tracker = cv2.TrackerMIL_create()
    Tracker.init(img, ROI)


def getInitialBounds(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    lower_blue = [110,50,50]
    upper_blue = [130,255,255]
    lb = (lower_blue[0], lower_blue[1], lower_blue[2])
    ub = (upper_blue[0], upper_blue[1], upper_blue[2])
    thresh = cv2.inRange(hsv, lb, ub)   # Find all pixels in color range

    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, _kernel)     # Open morph: removes noise w/ erode followed by dilate
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel)      # Close morph: fills openings w/ dilate followed by erode
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
    
    # no countours found
    if len(cnts) <= 0:
        return None

    # Get the largest contour area
    c = max(cnts, key=cv2.contourArea)

    # Filter out really small shapes
    if cv2.contourArea(c) < 200:
        return None
    
    # Get bounding box (x,y,w,h) of the largest contour
    roi = cv2.boundingRect(c)
    return roi


def getAngleCategory(angle):
    if 180 >= angle > 130:
        return "slightly left"
    elif 130 >= angle > 80:
        return "left"
    elif 80 >= angle > 30:
        return "hard left"
    elif 30 >= angle >= 0:
        return "pivot left"
    elif -180 <= angle < -130:
        return "slightly right"
    elif -130 <= angle < -80:
        return "right"
    elif -80 <= angle < -30:
        return "hard right"
    elif -30 <= angle <= 0:
        return "pivot right"
    else:
        return "straight"


def avoidObstacles(distance, angle):
    if 0.08 <= distance < 0.5:  # If the distance is less than 0.5 meters
        angle_category = getAngleCategory(angle)
        print(f"Distance: {distance}, Angle: {angle}, Action: {angle_category}")

        if angle_category == "slightly left":
            lm.sendLeft(1.0)
            lm.sendRight(0.8)
        elif angle_category == "left":
            lm.sendLeft(1.0)
            lm.sendRight(0.6)
        elif angle_category == "hard left":
            lm.sendLeft(1.0)
            lm.sendRight(0.4)
        elif angle_category == "pivot left":
            lm.sendLeft(1.0)
            lm.sendRight(0.0)
        elif angle_category == "slightly right":
            lm.sendLeft(0.8)
            lm.sendRight(1.0)
        elif angle_category == "right":
            lm.sendLeft(0.6)
            lm.sendRight(1.0)
        elif angle_category == "hard right":
            lm.sendLeft(0.4)
            lm.sendRight(1.0)
        elif angle_category == "pivot right":
            lm.sendLeft(0.0)
            lm.sendRight(1.0)
        return True
    elif distance < 0.0799999:
        lm.sendLeft(-0.8)
        lm.sendRight(-0.8)
        sleep(2)
        lm.sendLeft(0.0)
        lm.sendRight(0.8)
        sleep(2)
        return True
    return False


def main():
    initialBounds = None
    img = None
    while initialBounds is None:
        success, img = cap.read()
        if not success:
            print("Failed to read from camera!")
            continue
        initialBounds = getInitialBounds(img)
        print("Initial target not found!")
        print("Trying again in 0.5 seconds...")
        sleep(0.5)

    print("Starting bounds:", initialBounds)
    Tracker.init(img, initialBounds)

    while True:
        # Get LIDAR measurements to map the room
        ob_distance, ob_angle = lv.getNearest()
        print(f"Obstacle (r={ob_distance}, θ={ob_angle})")

        # First check for obstacles. If we're about to crash into something,
        # it doesn't matter if the human is right behind us.
        obstacleAvoided = avoidObstacles(ob_distance, ob_angle)
        if obstacleAvoided:
            sleep(0.1)
            continue

        Timer = cv2.getTickCount()
        success, img = cap.read()

        foundHuman, bound = Tracker.update(img)

        frame_height, frame_width, _ = img.shape
        x, y, w, h = bound

        if ((frame_height - x) < EDGE_MARGIN) or ((frame_width - y) < EDGE_MARGIN) \
            or (x < EDGE_MARGIN) or (y < EDGE_MARGIN):
            # The bounds are really close to the edge,
            # so we'll assume we lost the object.
            foundHuman = False

        if foundHuman:
            # Human detected!
            x,y,w,h = bound                                             # Get bounding rectangle (x,y,w,h) of the target
            center_x, center_y = (int(x+0.5*w), int(y+0.5*h))           # defines center of rectangle around the largest target area
            angle = round(((center_x / frame_width) - 0.5) * FOV, 3)    # angle of vector towards target center from camera, where 0 deg is centered

            print(f"Human found at {center_x}, {center_y}!")

            wheel_measured: np.array = kin.getPdCurrent()           # Wheel speed measurements

            # Always move away from the target
            fwd_effort = 2
            
            wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
            sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
            print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
        else:
            # SPIIIIIIIN!!!
            print("No targets")
            sc.driveOpenLoop(np.array([+4.0, -4.0]))
            restartTracking(img)


if __name__ == '__main__':
    main()
