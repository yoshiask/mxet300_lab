import cv2
import numpy as np

cap = cv2.VideoCapture(1)

Tracker = cv2.TrackerMIL_create()


def drawBox(img, bound):
    x, y, w, h = [int(i) for i in bound] #300, 300, 300, 300
    cv2.rectangle(img,(x,y), ((x+w), (y+h)), (255, 0, 0), 3, 1 ) 
    cv2.putText(img, "TRACKING", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    # print('x:', x)
    # print('y:', y)
    # print('w:', w)
    # print('h:', h)
    print(bound)


def Out_of_Frame():
    cv2.putText(img, "LOST", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    # ROI = cv2.selectROI("Tracking...", img, False)
    ROI = getInitialBounds(img)
    if ROI is None:
        return
    Tracker = cv2.legacy.TrackerMOSSE_create()
    Tracker.init(img,ROI)


def getInitialBounds(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 
    lower_blue = [110,50,50]
    upper_blue = [130,255,255]
    lb = (lower_blue[0], lower_blue[1], lower_blue[2])
    ub = (upper_blue[0], upper_blue[1], upper_blue[2])
    thresh = cv2.inRange(hsv, lb, ub)   # Find all pixels in color range

    kernel = np.ones((5,5),np.uint8)                            # Set kernel size
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
    
    if len(cnts) <= 0:                                      # no countours found
        return None

    c = max(cnts, key=cv2.contourArea)                      # return the largest target area
    if cv2.contourArea(c) < 200:                             # Filter out really small shapes
        return None
    roi = cv2.boundingRect(c)   # Get bounding rectangle (x,y,w,h) of the largest contour
    return roi


success, img = cap.read()
#ROI = cv2.selectROI("Tracking...", img, False)

roi = getInitialBounds(img)
print("Starting contour:", roi)

if roi is None:
    print("Initial target not found!")
    exit(-1)

Tracker.init(img, roi)

while True:
    Timer = cv2.getTickCount()
    success, img = cap.read()

    success,bound = Tracker.update(img)

    frame_height, frame_width, _ = img.shape
    x, y, w, h = bound

    EDGE_MARGIN = 5
    if ((frame_height - x) < EDGE_MARGIN) or ((frame_width - y) < EDGE_MARGIN) \
        or (x < 0) or (y < 0):
        # The bounds are really close to the edge,
        # so we'll assume we lost the object.
        success = False

    if success:
        # Captures the live stream frame-by-frame 
        _, frame = cap.read()  
        # Converts images from BGR to HSV 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
        lower_blue = np.array([110,50,50]) 
        upper_blue = np.array([130,255,255]) 
        drawBox(img, bound)
        mask = cv2.inRange(hsv, lower_blue, upper_blue) 
  
        res = cv2.bitwise_and(frame,frame, mask= mask) 
        
        # This displays the frame, mask  
        # and res which we created in 3 separate windows. 
        #cv2.imshow('frame',frame) 
        cv2.imshow('mask',mask) 
        cv2.imshow('res',res) 
  
        # k = cv2.waitKey(5) & 0xFF
        # if k == 27: 
        #     break
    elif not success: 
        Out_of_Frame()
    

    cv2.imshow("Tracking...", img)
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-Timer)
    cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # Destroys all of the HighGUI windows. 
    #cv2.destroyAllWindows() 
    
    # release the captured frame 
    #cap.release() 
    if cv2.waitKey(1) & 0xff ==ord('q'):
        break

