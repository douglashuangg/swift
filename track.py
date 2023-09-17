import cv2
import serial
import time

#Video Source
cap = cv2.VideoCapture(0)

# set serial port
ser = serial.Serial('COM5', 115200)
if not ser.is_open:
    ser.open()

# set past coordinates
prev_x = 0
prev_y = 0
y_total_angle = 90

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
    cv2.putText(img,str(cy),(150,75),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,255,0),2)

    # Y co-ordinates
    return cx, cy

while True:
    cx = 0
    cy = 0
    timer = cv2.getTickCount()
    # Displaying sucess if object detected
    success, img = cap.read()

    success, bbox = tracker.update(img)

    # Displaying if object is detected or lost
    if success:
        cx, cy = drawBox(img,bbox)
        if prev_x == 0 and prev_y == 0:
            prev_x = cx
            prev_y = cy
    else:
         cv2.putText(img, "Lost",(75,50),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,0,255),2)

    # Getting fps
    #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
    #cv2.putText(img,str(int(fps)),(75,50),cv2.FONT_HERSHEY_TRIPLEX,0.7,(0,0,255),2)
    cv2.imshow("Tracking", img)

    # sending data to arduino
    angle = (cx - prev_x)/320 * 45
    y_angle = (cy - prev_y)/220 * 25
    # if(angle > 10 or angle < -10):
    #     coord_str = f"{angle}, {y_angle}\n"
    #     print(angle)

    #     ser.write(coord_str.encode())
    #     prev_x = cx

    if(y_angle > 3 or y_angle < -3):
        if(y_angle < 14 and y_angle > -14):
            y_total_angle -= y_angle
            coord_str = f"{y_total_angle}\n"
            print(y_total_angle)
            print("angle", y_angle)

            ser.write(coord_str.encode())
            prev_y = cy
  

    if ser.in_waiting > 0:
        data = ser.readline()
        print("Received from Arduino:", data)


    # Press 'q' to terminate program
    if cv2.waitKey(1) & 0xff == ord('q'):
        ser.send_break()
        ser.close()
        break

# Program end statements
cap.release()
cv2.destroyAllWindows()
