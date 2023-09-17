import cv2

#Video Source laptop camera
cap = cv2.VideoCapture(0)

#Video Source webcam
#cap = cv2.VideoCapture(1)

# mosse tracker
#Low accuracy, high speed
#tracker = cv2.legacy.TrackerMOSSE_create()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           b
#High accuracy, low speed
tracker = cv2.TrackerCSRT_create()

#Bounding box selection
success, img = cap.read()
bbox = cv2.selectROI("Tracking",img,False)
tracker.init(img,bbox)

# constant tracking box
def drawBox(img, bbox):
    # Taking the distance, height and width using bbox
    x,y,w,h = int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
    cv2.rectangle(img,(x,y),((x+w),(y+h)),(255,0,255),3,1)

    #Finding the middle for tracking
    success,box=tracker.update(img)
    if success:
        (x,y,w,h)=[int(a)for a in box]
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        cx = (x + x + w) // 2
        cy = (y + y + h) // 2
        cv2.circle(img,(cx,cy),5,(0,0,255),-1)
        cv2.line(img,(cx,0),(cx,480),(0,0,255),2)
        a=int(cx)//65
    #Display if tracking
    cv2.putText(img, "Tracking",(75,50),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,255,0),2)
    # X co-ordinates
    cv2.putText(img,str(cx),(75,75),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,255,0),2)
    # Y co-ordinates
    cv2.putText(img,str(cy),(150,75),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,255,0),2)

while True:
    timer = cv2.getTickCount()
    # Displaying sucess if object detected
    success, img = cap.read()

    success, bbox = tracker.update(img)

    # Displaying if object is detected or lost
    if success:
        drawBox(img,bbox)
    else:
         cv2.putText(img, "Lost",(75,50),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,0,255),2)

    # Getting fps
    #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
    #cv2.putText(img,str(int(fps)),(75,50),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,0,255),2)
    cv2.imshow("Tracking",img)

    # Press 'q' to terminate program
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

# Program end statements
cap.release()
cv2.destroyAllWindows()
