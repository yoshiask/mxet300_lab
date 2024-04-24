import cv2

cap = cv2.VideoCapture(1)

Tracker = cv2.legacy.TrackerCSRT_create()
success, img = cap.read()
bound = cv2.selectROI("Tracking...", img, False)
Tracker.init(img,bound)

def drawBox(img, bound):
    x, y, w, h = int(bound[0]), int(bound[1]),int(bound[2]), int(bound[3])
    cv2.rectangle(img,(x,y), ((x+w), (y+h)), (255, 0, 0), 3, 1 ) 
    cv2.putText(img, "TRACKING", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

while True:
    Timer = cv2.getTickCount()
    success, img = cap.read()

    success,bound = Tracker.update(img)

    if success:
        drawBox(img, bound)
    else: 
        cv2.putText(img, "LOST", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imshow("Tracking...", img)
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-Timer)
    cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    if cv2.waitKey(1) & 0xff ==ord('q'):
        break

