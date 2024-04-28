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

# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
#    Camera
stream_ip = getIp()
if not stream_ip: 
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

#    Color Range, described in HSV
v1_min = 70     # Minimum H value
v2_min = 80     # Minimum S value
v3_min = 180    # Minimum V value

v1_max = 105    # Maximum H value
v2_max = 255    # Maximum S value
v3_max = 255    # Maximum V value

target_width = 100      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 10       # Minimum width error to drive forward/back

def drawBox(img, bound):
    x, y, w, h = int(bound[0]), int(bound[1]),int(bound[2]), int(bound[3])
    cv2.rectangle(img,(x,y), ((x+w), (y+h)), (255, 0, 0), 3, 1 ) 
    cv2.putText(img, "TRACKING", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)


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
