#Import modules and libraries
from cvzone.HandTrackingModule import HandDetector
import cv2 
from keyboard import is_pressed as pressed
import numpy as np
from time import monotonic
import abc
import PySimpleGUI as sg
#Video Class that initializes a OpenCV video feed and updates it
class Video():
    def __init__(self, name = "Image"):
        #connect to default camera
        self.capture = cv2.VideoCapture(0)
        #capture name (Deault = "Image")
        self.name = name
        #Grab frame and display it
        self.update()
        self.display()

    #function that updates frame from video feed and displays
    def update(self):
        #Grab frame
        self.success, self.img = self.capture.read()

    def display(self, img):
        #display frame
        cv2.imshow(self.name, img)
        cv2.waitKey(1)      

#American Sign Language Detector Superclass
#Includes functions to calibrate, get measurements, and calculate other values based on video feed 
class ASLDetector(metaclass = abc.ABCMeta):
    
    #Initilization, passes a default generic videofeed
    #Initializes other varibles
    def __init__(self, videoFeed = Video(), draw = True):
        self.handDetector = HandDetector(detectionCon=0.8, maxHands=1)
        self.delayTimerEndTime = 0
        self.DEBUG = True
        self.draw = draw 
        self.videoFeed = videoFeed
        self.measurementPairs = [
            
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

    ##Accessor for the Draw variable findHands
    @property
    def draw(self):
        return self._D
    ## mutator for the Draw variable in findHands
    @draw.setter
    def draw(self, value):
        self._D = value       
    
    #Function that detects the right hand from video feed
    def findHands(self):
        self.hands = self.handDetector.findHands(self.videoFeed.img, draw=self.draw) 
        if self.hands:
            if self.hands[0]["type"] == "Right":
                self.hand = self.hands[0]
            else:
                self.hand = None
        else:
            self.hand = None

    #Function to calibrate the focal length of the camera in order to find distance between hand and camera
    def calibrate(self):
        
    #Get measurement set defined by measurement pairs in __init__ and angle of hand
    def getMeasurements(self):
        self.findHands()
        self.measurements = []
        if self.hand:
            for index, pair in enumerate(self.measurementPairs):
                x, y, bit = pair
                measurement, info = self.handDetector.findDistance(self.hand["lmList"][x], self.hand["lmList"][y])
    
    #Set input timer end time by adding a delay to the current time reading
    def startInputTimer(self, delayTime = 1.0):
        self.delayTimerEndTime = monotonic() + delayTime
    
    #check if current time is greater than the generated time by startInpuDelayTimer: return False if greater
    def inputTimer(self):
        if monotonic() > self.delayTimerEndTime:
            return False
        else:
            return True
    
    #Function that takes in a matrix and creates a list of pairs or averages from the matrix 
    def createMeasurementRange(self, matrix, pairs = True):
        outputList = []
        
        for i in range(len(matrix[0])):
            x = matrix[0][i]
            y = matrix[1][i]
            
            if x > y:
                val = (x,y)
            else:
                val = (y,x)
            outputList.append(val)
            
        for measurementSet in matrix:
            for index, measurement in enumerate(measurementSet):
                min, max = outputList[index]
                if measurement> max:
                    max = measurement
                elif measurement < min:
                    min = measurement
                outputList[index] = (min,max)
                
        if self.DEBUG:
            for min, max in outputList:
                print(f"max:{max}\tmin{min}\trange:{max-min}")
                
            print(outputList)
        
        if pairs:
            return outputList
        else:
            return [(min+max)/2 for min, max in outputList]
    
    #method to be defined by the detection method
    @abc.abstractmethod
    def detectLetter(self):
        raise NotImplementedError("Not Supported")

#detection method based on detecting touching points
#the dictionary has hexadecimal keys that relate to an 8 bit binary string that encodes the measurements
class TouchDetector(ASLDetector):
    def __init__(self):
        #initalize superclass
        super().__init__()

        #Pairs of points detector checks are touching and the bit in hex they affect
        self.measurementPairs = [[(8,12,0x80), (12,16,0x40), (16,20,0x20)]]

        #Keys are in hex and encode the following information
        #if 8 & 12 are touching
        #if 12 & 16 are touching
        #if 16 & 20 are touching
        #what point 4 is touching, 0 is None and 21 is center
        self.dictionary = {
            0xE6 : "A",
            0xF5 : "B",
            0xE0 : "C",
            0x6C : "D",
            0xF4 : "E",
            0x08 : "F",
            0x68 : "G",
            ##0x : "H",
            0xC7 : "I",
            0x45 : "K",
            0x60 : "L",
            0xD3 : "M",
            0xAA : "N",
            0xE4 : "O",
            0x2A : "P",
            0x68 : "Q",
            0xB0 : "R",
            0xEC : "S",
            0x6A : "T",
            0xAE : "U",
            0x30 : "V",
            0x14 : "W",
            0x6D : "X",
            0xC0 : "Y",
        }

        #Function to calibrate the distance threshold for points touching
    def calibrate(self):
        
        #initialize calibration list and actual value
        calibrationMatrix = []
        sum = 0
        
        #print instructions for user
        print("Hold your right hand near your chest, index , middle and thumb touching")
        print("Please hold position for approximately 1 second")
        print("Press space when ready")
        
        #Wait for user to press Space
        while not pressed("space"):
            self.videoFeed.update()
            self.videoFeed.display()
        
        #Print hold and dots while 20 measurements are being taken
        #measurement is between base of hand and base of thumb
        print("Hold")
        while len(calibrationMatrix) < 20:
            self.videoFeed.update
            self.findHands()
            if self.hands:
                measurementSet = [
                    self.handDetector.findDistance(self.hand['lmList'][4], self.hand["lmList"][8]),
                    self.handDetector.findDistance(self.hand['lmList'][4], self.hand["lmList"][12]),
                    self.handDetector.findDistance(self.hand['lmList'][8], self.hand["lmList"][12]),
                    ]
                calibrationMatrix.append(measurementSet)
            print(".")
            self.videoFeed.display() 
        
        #Notify user when done
        print("DONE")
    
        
        #Average the measurements
        for measurementSet in calibrationMatrix:
            for measurement, info in measurementSet:
                sum += measurement
        average = sum / len(calibrationMatrix)*3
        
        #Set touch trheshold to average measurement
        self.touchThreshold = average
        
    def isTouching(self, p1, p2):
        #find distance between points
        measurement, info = self.handDetector.findDistance(p1, p2)

        #if distance is less than touch threshold they are touching
        if measurement < self.touchThreshold:
            return True
        #otherwise they are not
        else: 
            return False
    
    #Function that will take nessesary measurements and encode the measurements into hex
    def getMeasurements(self):

        #if hand is detected
        self.findHands()
        if self.hand:

            #add center to the end of the landmark list
            self.hand["lmList"][21] = self.hand["center"]

            #reset measurement base
            measurements = 0x00
            
            #for the point pairs see if they are touching and change the corresponding bit appropriately
            for pair in self.measurementPairs:
                x, y, bit = pair
                if self.isTouching(self.hand["lmList"][x], self.hand["lmList"][y]):
                    measurements = measurements | bit
            
            #determine which point the thumb is touching
                for point in self.hand["lmList"]:
                    if self.isTouching (self.hand["lmList"][4], self.hand["lmList"][point]):
                        measurements = measurements | point
            
            return measurements
                
    def detectLetter(self, code):
        try:
            print(self.dictionary[code])
        except:
            pass


######################################################################################
                        #       PySimpleGUI     #
######################################################################################
class GUI():

    def __init__(self):
        pass
    ## creates a variable for our main detector that can easily be called
    detector = ComparisonDetector()
    # changing the color theme of GUI, PySimpleGui has hundreds if we ever want to change easily
    sg.theme('LightBlue')   
    #text layout along with buttons
    ## the header text within the GUI
    welcomeText     =[[sg.Text("ASL Detector Alpha", size =30,  font='Helvetica 20')]]
    ## button that accesses the calibration function
    calibrateButton =[[sg.Button("Calibrate", key="calibrate", size = 20)]]
    ## button that changes the variable for drawing the landmarks/ traces on hands in the video image
    showlinesButton =[[sg.Button("Show Lines", key="showlines", size = 20)]]
    ## button that will toggle the LED functionality
    toggleLED       =[[sg.Button("Toggle LED Light", key="led", size = 20)]]
    ## button that will toggle the auditory reading of detected letters
    toggleSpeaker   =[[sg.Button("Toggle Speaker", key="speaker", size = 20)]]
    ## a button with no implementation currently
    extraButton     =[[sg.Button("Extra", key="extra", size = 20)]]
    ## button that serves the same purpose as the "x" in the upper right window
    exitButton      =[[sg.Button("Exit", key = "close", size = 20)]]
    ## a text output that takes in the detected letters as an argument and prints them below the image output 
    detectedLetters =[[sg.Text(f"{detector.detectLetter()},")]]
    ## the actual videofeed captured and returned into a frame on the GUI
    liveCamera      =[[sg.Image(filename='', key='image')]]
    #liveCamera     =[[sg.Image(detector.videoFeed.update())]]


    ### expiremental layouts that I am currently trying to get PySimpleGUI to work how I want it to

    # buttonColumn    =[[sg.Button("Calibrate", key="calibrate", size = 20)],
    #                  [sg.VPush()],
    #                  [sg.Button("Show Lines", key="showlines", size = 20)],
    #                  [sg.VPush()],  
    #                  [sg.Button("Toggle LED Light", key="led", size = 20)],
    #                  [sg.VPush()],
    #                  [sg.Button("Toggle Speaker", key="speaker", size = 20)],
    #                  [sg.VPush()],
    #                  [sg.Button("Extra", key="extra", size = 20)],
    #                  [sg.VPush()],
    #                  [sg.Button("Exit", key = "close", size = 20)]]

    ### Right most column that will contain video feed, detected text, and the title 

    # TitleVideoColumn=[[sg.Text("ASL Detector Alpha", size =30,  font='Helvetica 20')],
    #                   [sg.Image(filename = '', key ="image")],
    #                   [sg.Text(f"{detector.detectLetter()},")]]

    ## the layout for PySimpleGui, Each seperate line in the code is a different row, items in the same row with commas in-between are columns
    ## Vpush is a verticle seperation, Push is horizontal. Vseperator is a line to divide columns'
    ## 
    layout = [  [sg.Column(welcomeText),sg.Push(),sg.Image(filename='', key='image')],
                [calibrateButton],
                [showlinesButton], 
                [liveCamera],
                [toggleLED],
                [toggleSpeaker],
                [extraButton],
                [exitButton]]

    #alternative layout, I (matt) am tweaking with
    #layout = [[sg.Column(buttonColumn),sg.VSeperator(),sg.Push(),sg.Column(TitleVideoColumn)]]

                

    # Create the Window
    window = sg.Window('Settings', layout, size = (1000,800))
    cap = cv2.VideoCapture(0)
    # Event Loop to process "events" 
    while True:
        event, values = window.read(timeout=20)
        ret, frame = cap.read()
        imgbytes = cv2.imencode('.png', frame)[1].tobytes()  # ditto
        window['image'].update(data=imgbytes)
        ## if the close button or the x in the top right of the window is pressed this will Close the GUI 
        if event in (sg.WIN_CLOSED, 'close'):
            break
        ## calls the calibrate function that is defined within the ASL detector class 
        elif event == "calibrate":
            detector.calibrate()
        ## when the showlines button is clicked this will change the instance variable of the findHands function to be True if it was False and Vice- Versa
        ## I am not sure if this is the right way to call this(it definetely isnt but I could no get the draw parameter to actually change anything) but I am pretty sure that it is close
        elif event == "showlines":
            print(ASLDetector.draw)
            if(ASLDetector.draw == True ):
                ASLDetector.draw = False
            else:
                ASLDetector.draw=True
        ## unused button as of now but I was thinking maybe have it toggle a box or something that shows the distance of the hand from the camera
        ## or we could use it to test accuracy at different lengths and alternatively/ for more consistency could try to get users to try from the same Distance 
        elif event == "extra":
            pass
        ## this will just toggle the speaker from whatever GPIO pin that we are using connected to the raspPI
        elif event =="speaker":
            pass
        ## this will just toggle the LED from whatever GPIO pin that we are using connected to the raspPI
        elif event =="led":
            pass
        measurementMatrix = []

        if pressed("R"):
            measurementMatrix.append(detector.measurements)
            list = detector.createMeasurementRange(measurementMatrix)
            print (list)

def main():
    g = GUI()

    
if __name__ == "__main__":
    main()
