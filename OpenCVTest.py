#import required libraries
from cvzone.HandTrackingModule import HandDetector
import cv2
from time import monotonic
import numpy as np

#Global Variables
#Debug Variable
DEBUG = False
##Timer Related Variables
delayTimerEndTime = 0

#List of point pairs to measure
measurementPairs = [
    
    #Connect Neighbors
    (4, 8), #Thumb tip to index tip
    (8, 12), #Index tip to middle tip
    (12, 16), #Middle tip to ring tip
    (16, 20), #Ring tip to pinky tip
   
    #Connect tips to bottom of hand
    (4, 0), #Thumb to bottom of hand
    (8, 0), #Index to bottom of hand
    (12, 0), #Middle to bottom of hand
    (16, 0), #Ring to bottom of hand
    (20, 0), #Pinky to bottom of hand
   
    #Connect thumb tip to other tips
    (4, 12), #Thumb to middle
    (4, 16), #Thumb to ring
    (4, 20) #Thumb to pinky

]

#Dictionary of letters represented by their corresponding 
#measurement value list
alphaDict = {
    
    "A" : [75,25,30,30,180,100,85,80,80,100,115,130,45]
    
}

#function that finds the angle of the hand
def findAngle(hand, img = None):
    
    #base and center landmarks
    baseX, baseY, z = hand["lmList"][0]
    middleFingerX, middleFingerY, z = hand["lmList"][9]
    
    #define vertical line
    vert = (baseX, 0)
    
    #create vectors
    baseToPalm = np.array([middleFingerX, middleFingerY]) - np.array([baseX, baseY])
    baseToTop = np.array(vert) - np.array([baseX, baseY])
    
    #calculate angle
    angle = np.degrees(np.math.atan2(np.linalg.det([baseToPalm, baseToTop]), np.dot(baseToPalm, baseToTop)))
    
    #if image is passed, print the vectors and return value
    if img is not None:
        cv2.line(img, (baseX,baseY), (middleFingerX, middleFingerY), (225,225,0), 3)
        cv2.line(img, (baseX,baseY), (vert[0],vert[1]), (225,225,0), 3)
        return angle, img
    
    #otherwise just return value
    else:
        return angle                

#Function that checks the input timer
def inputDelayTimer():
    if monotonic() > delayTimerEndTime:
        return True
    else:
        return False

#Function that sets the input delay timer
def startInputDelayTimer(delayTime):
    global delayTimerEndTime
    delayTimerEndTime = monotonic() + delayTime

#Function to get mesurements (defined by measurementPairs)
#and angle
def getMeasurements(hand, img = None):
    #reset measurement array
    measurements = []
                
    #measure distances
    for x, y in measurementPairs:
        measurement, info = detector.findDistance(rightLandmarks[x], rightLandmarks[y])
        measurements.append(measurement)
    
    #if debug is on, print values and display angle lines
    if DEBUG and img is not None:
        angle, img = findAngle(rightHand, img)
        measurements.append(angle)      
        #print all measurements
        print (f"measurements:{measurements}\nangle{angle}")
        return measurements, img
    
    else:
        #Calculate angle of hand and add to measurement list
        angle = findAngle(rightHand)
        measurements.append(angle)
        return measurements
   
#initialize the video capture and hand detector
cap = cv2.VideoCapture(0)
detector = HandDetector(detectionCon=0.8, maxHands=1)

#Operational Loop
while True:
    
    #Capture video and find hands
    success, img = cap.read()
    hands, img = detector.findHands(img, draw=True)
    
    #If hand is detected
    if hands:
        
        #Check if it is the right hand
        if hands[0]["type"] == "Right":
            
            #Define hand and landmarks
            rightHand = hands[0]
            rightLandmarks = rightHand["lmList"]
            rightCenter = rightHand["center"]
            
            #check input delay timer
            if inputDelayTimer():
                
                #measure hand, print if debug
                if not DEBUG:
                    measurements = getMeasurements(rightHand)
                else:
                    measurements, img = getMeasurements(rightHand, img)
                
                try:
                    #for each letter in the letter list
                    for letter in alphaDict:
                        #for each value in measurements
                        for pos, val in enumerate(measurements):
                            #check if the value is less than the value 
                            #in the matching position in the letter
                            if val > float(alphaDict[letter][pos]):
                                break
                        
                            #if the last check passes print the letter 
                            #and delay input for 10 seconds
                            elif pos == len(measurements) - 1:
                                print(letter)
                                startInputDelayTimer(10)
                                
                                #break loop
                                raise StopIteration
                            
                except StopIteration:
                    pass

    #Show image
    cv2.imshow("Image", img)
    cv2.waitKey(1)
