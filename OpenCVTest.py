#import required libraries
from cvzone.HandTrackingModule import HandDetector
import cv2
from time import monotonic
import matplotlib.pyplot as plot

#Define Constants
##delay between each input can be accepted 
INPUT_DELAY = 5

##Defalut value for distance -- ONLY USED BECAUSE CODE IS WRITTEN POORLY
DEFAULT_VAL = 34_000

#Initialize the video capture and hand detector
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8,maxHands=1)

#Iititalize varibles
pauseEnd = 0
pauseStart = 0
fingers1 = []
fingers2 = []
thumbToJoint2Index = DEFAULT_VAL

xValues = []
yValues = []
fingerTips = [4,8,12,16,20]
palmBase = 0
firstKnuckle = [2,5,9,13,17]

#Operational Loop
while True:
    
    #Capture Video and find hands
    success, img = cap.read()
    hands,img = detector.findHands(img, draw=True)
    
    #If it detects a hand
    if hands:
        # Hand 1
        hand1 = hands[0]
        
        #create matrix of points on the hand
        lmList1 = hand1["lmList"]
        
        #detect how many fingers are up 
        #translated into a list from thumb to pinky; 1 is up and 0 is down
        fingers1 = detector.fingersUp(hand1)
        
        hand1Center = hand1["center"]
        
        #unpack position of index fingertip
        x,y,z = lmList1[fingerTips[1]]
        #take the point and add them to the coodinate lists
        xValues.append(x)
        yValues.append(y)
        
        #measure distance between thumb tip and middle tip
        length, info = detector.findDistance(lmList1[fingerTips[0]], lmList1[fingerTips[2]])
        #if they are touching, leave operational loop
        if length < 20:
            break
        
        #measure thumb tip to bottom joint of the index finger
        thumbToJoint2Index, info = detector.findDistance(lmList1[4], lmList1[6])
        
        #measure the thumb tip to center of palm
        thumbToPalm, info = detector.findDistance(lmList1[4], hand1Center)
        
        #If thumb is up and all other fingers are down and thumb is touching bottom index joint
        if fingers1 == [1,0,0,0,0] and thumbToJoint2Index < 20 and input is False:
            
            #print "A"
             print ("A")
             
             #Start input delay timer
             input = True
             pauseEnd = monotonic() + INPUT_DELAY

        #If thumb is down and all other fingers are up and thumb is touching center of palm
        elif fingers1 == [0,1,1,1,1] and thumbToPalm < 20:
            
            #print B
            print ("B")
            
            #Start input delay timer
            input = True
            pauseEnd = monotonic() + INPUT_DELAY
            
        #If input timer is running and full delay has elapsed
        #End delay timer and reset input
        elif monotonic() >= pauseEnd:
            input = False
            print("input reset")
    
    #display image and cleanup delay
    cv2.imshow("Image", img)
    cv2.waitKey(1)
    
#Cleanup
cap.release()
cv2.destroyAllWindows()

#plot and display the values the index finger tip took
plot.plot(xValues, yValues)
plot.show()
