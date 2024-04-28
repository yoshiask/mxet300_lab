# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
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


initialBounds = None
while initialBounds is None:
    success, img = cap.read()
    if not success:
        print("Failed to read from camera!")
        continue
    initialBounds = getInitialBounds(img)
    print("Initial target not found!")
    print("Trying again in 5 seconds...")
    sleep(5)

print("Starting bounds:", initialBounds)
Tracker.init(img, initialBounds)

while True:
    Timer = cv2.getTickCount()
    success, img = cap.read()

    success, bound = Tracker.update(img)

    frame_height, frame_width, _ = img.shape
    x, y, w, h = bound

    if ((frame_height - x) < EDGE_MARGIN) or ((frame_width - y) < EDGE_MARGIN) \
        or (x < EDGE_MARGIN) or (y < EDGE_MARGIN):
        # The bounds are really close to the edge,
        # so we'll assume we lost the object.
        success = False

    if success:
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


def main():
    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    #camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    #camera.set(4, size_h)                       # Set height of images that will be retrived from camera

    Tracker = cv2.legacy.TrackerCSRT_create()
    success, img = camera.read()
    bound = cv2.selectROI("Tracking...", img, False)
    Tracker.init(img,bound)

    try:
        while True:
            sleep(.05)                                          

            Timer = cv2.getTickCount()
            success, img = camera.read()
            success, bound = Tracker.update(img)

            if success:
                drawBox(img, bound)
            else: 
                cv2.putText(img, "LOST", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            cv2.imshow("Tracking...", img)
            fps = cv2.getTickFrequency()/(cv2.getTickCount()-Timer)
            cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            
            if success:
                # Human detected!
                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured: np.array = kin.getPdCurrent()           # Wheel speed measurements

                # Always move away from the target
                fwd_effort = 2
                
                wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
            else:
                # TODO: SPIIIIIIIN!!!
                print("No targets")
                sc.driveOpenLoop(np.array([+4.0, -4.0]))
            
            if cv2.waitKey(1) & 0xff ==ord('q'):
                break

                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
    	print("Exiting Color Tracking.")

if __name__ == '__main__':
    main()
