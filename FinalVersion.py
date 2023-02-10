#Import modules and libraries
from cvzone.HandTrackingModule import HandDetector
import cv2
from time import monotonic
import PySimpleGUI as sg
from math import hypot
#import RPi.GPIO as GPIO
import os
import pygame
from PIL import Image
pygame.init()
os.chdir(os.path.dirname(os.path.abspath(__file__)))
#screen dimensions
w, h = sg.Window.get_screen_size()
# LED and Speaker 
LED_PIN = 17
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(LED_PIN, GPIO.OUT)

class Video():
    def __init__(self, name = "Image"):
        #connect to default camera
        self.capture = cv2.VideoCapture(0)
        #self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        #capture name default = Image
        self.name = name

    #updates img with a new frame
    def update(self):
        _, self.img = self.capture.read()

class GUI():
    def __init__(self):
        self.toggle = True
        

    def home(self):
        global window
        #used to place elements that are invisible in order to maintain the layout
        #when their visibility is updated
        def place(elem):
            return sg.Column([[elem]], pad=(0,0))
        # changing the color theme of GUI    
        sg.theme('DarkBlue')   
        # camera feed on the GUI
        cameraRow   = [[sg.Push(),sg.Image(filename='', key='image', size=(10,10)),sg.Push()]]
        #the first row of buttons, containing the Calibrate button, ASL Chart, and LED checkbox
        buttonRow1  = [[sg.Push(),sg.Text("      ", size=8),sg.Button("Calibrate", key="calibrate", size = 20),sg.Button("ASL Chart", key="asl", size = 20),sg.Checkbox("Toggle LED Light",default=True, key="led"),sg.Push()]]
        #the second row of buttons, Clear Input, Exit, and Speaker Checkbox.
        buttonRow2  = [[sg.Push(),sg.Text(" ", size=7),sg.Button("Clear Input", key="clear", size = 20),sg.Button("Exit", key = "exit", size = 20),sg.Checkbox("Toggle Speaker",default=False, key="speaker"),sg.Push(),]]
        #header text of the GUI, the Title
        welcomeText = [[sg.Push(),sg.Text("      ", size=19),sg.Text("ASL Detector Alpha", size =30,  font='Helvetica 20'),sg.Push()]]

        #the updated output string that is produced by the detected letters 
        detectedletterString = [[sg.Output(size=(80,10), key="output")]]

        # the formatting for the layout of the GUI 
        layout =  [[welcomeText],
                   [cameraRow],
                   [buttonRow1],
                   [buttonRow2],
                   [place(sg.Button("", key="LED", size = 20, button_color = (sg.theme_background_color(), sg.theme_background_color()),image_filename='led.png', visible = not self.toggle))],
                   [detectedletterString]]
        

        window = sg.Window('ASL Detector', layout,element_justification='c', size = (w,h))

    #updates the GUI video with the most recent captured frame
    #converts the frame to a bytestream so that PySimpleGUI can interpret it
    #delay key required
    def updateImg(self, img):
        imgbytes = cv2.imencode('.png', img)[1].tobytes()  # ditto
        window['image'].update(data=imgbytes)
        cv2.waitKey(1)



class TouchDetector():
    def __init__(self, gui = GUI(), videoFeed = Video()):
        """
        Initializes a ASL Detector using the touch detection method

        Parameters:
        gui (object): Instance of GUI class
        videoFeed (object): Instance of Video class
        """

        self.videoFeed = videoFeed
        self.gui = gui
        self.DEBUG = True
        self.draw = False
        self.handDetector = HandDetector(detectionCon = 0.8, maxHands = 1)
        self.outputStr = ""
        self.delayTimerEndTime = 0
        self.touchThreshold = 20
        
    def findDistance(self, p1, p2):
        """
        finds the distance between 2 points in 2D space

        Uses the math hypot function to determine distance

        Parameters:
        p1 (float, float): first point
        p2 (float, float): second point

        Returns:
        float: The distance between point 1 and point 2

        """
        x1, y1 = p1
        x2, y2 = p2
        length = hypot(x2 - x1, y2 - y1)
        return length

    def calibrate(self):
        """
        Calibrates the touch threshold of the program

        Prompts the user to touch their index, middle and thumb together
        then measures the distances between each of the fingertips 20 times
        then averages the largest of each set to become the touch treshold

        Parameters:
        None
        Returns:
        None
        """
        calibrationMatrix = []
        sum = 0

        #########################################################
        #   INSTRUCTION POP-UP & VIDEO FEED HOLDING PATTERN     #
        #########################################################

        instrctionPopUP = sg.popup_ok("On your right hand combine your thumb, pointer and middle finger into a group and hold them in front of the camera.", keep_on_top=True)
        
        while len(calibrationMatrix) < 20:
            print("inLoop")
            self.videoFeed.update()
            self.gui.updateImg(self.videoFeed.img)
            self.findHands()
            if self.hand:
                measurementA  = self.findDistance(self.hand['lmList'][4], self.hand["lmList"][8])
                measurementB = self.findDistance(self.hand['lmList'][4], self.hand["lmList"][12])
                measurementC = self.findDistance(self.hand['lmList'][8], self.hand["lmList"][12])
                calibrationMatrix.append([measurementA, measurementB, measurementC])
                ##############################
                #    Display frame on GUI    #
                #    Progress bar            #
                ##############################
                sg.one_line_progress_meter("Calibration Completion", len(calibrationMatrix), 20)
            
        #Average the measurements
        for measurementSet in calibrationMatrix:
            sum += max(measurementSet)
        average = sum / len(calibrationMatrix)
        
        #Set touch trheshold to average measurement
        self.touchThreshold = average

        sg.popup("Done")

        #####################
        #    Done Pop-up    #
        #####################

    def isTouching(self, p1, p2):
        """
        Determines if point 1 and point 2 are touching

        Finds the distance between the 2 points then sees if the distance is less than the touch threshold

        Parameters:
        p1 (float, float): point 1
        p2 (float, float): point 2

        Returns:
        bool: If the points are touching
        """
        #find distance between points
        measurement = self.findDistance(p1, p2)

        #if distance is less than touch threshold they are touching
        if measurement < self.touchThreshold:
            return True
        #otherwise they are not
        else: 
            return False
        
    #Function that will take nessesary measurements and encode the measurements into hex
    def getMeasurements(self):
        """
        Generates a hex value based on measurements of the hand

        Determines if index and middle; middle and ring; ring and pinky are touching.
        Then based on the resulting number checks which point the thumb is touching

        Parameters:
        None

        Returns
        int: measurements encoded into a 2 digit hex value
        """


        primaryMeasurementPairs = [(8,12,0x80), (12,16,0x40), (16,20,0x20)]
        
        secondaryMeasurementPoints = {
            0x00 : [8, 20],
            0x20 : [5, 6, 16],
            0x60 : [12, 8, 10, 7],
            0xA0 : [10, 16, 18],
            0xC0 : [7, 18],
            0xE0 : [6, 21, 20, 8, 14]
        }

        #if hand is detected
        self.findHands()
        if self.hand:

            #add center to the end of the landmark list
            self.hand["lmList"].append(self.hand["center"])

            #reset measurement base
            measurements = 0x00
            
            #for the point pairs see if they are touching and change the corresponding bit appropriately
            for pair in primaryMeasurementPairs:
                x, y, bit = pair
                print(f"measurment for bit {hex(bit)} is")
                print( self.findDistance(self.hand["lmList"][x], self.hand["lmList"][y]))
                if self.isTouching(self.hand["lmList"][x], self.hand["lmList"][y]):
                    measurements |= bit
            
            #determine which point the thumb is touching
            for point in secondaryMeasurementPoints[measurements]:
                if self.isTouching (self.hand["lmList"][4], self.hand["lmList"][point]):
                    measurements |= point
                    break

            return measurements
                
    def detectLetter(self):
        """
        Uses hex code from getMeasurements to see if a valid letter was detected then outputs it

        If the hex code is valid, it will print to the GUI and play sound if sound is on
        If the hex code is invalid, it will pass

        Parameters:
        None

        Returns
        None

        """
        translationDictionary = {
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
        try:
            #####################################################################################
            #    Concatenate translationDictionary[self.getMeasurements()]) to output string    #
            #####################################################################################
            print("an attempt was made")
            print(self.getMeasurements())
            detectedLetters = translationDictionary[self.getMeasurements()]
            self.startInputTimer(2.5)
            self.outputStr+= detectedLetters
            window['output'].update('')
            print(self.outputStr)

            #creates a string variable with the detected letter, and adds a .wav file type to the end
            ihatepygame = (f"{translationDictionary[self.getMeasurements()]}.wav")
            #checks to see if the checkbox is selected
            if values["speaker"] == True:
                #creates a temporary variable for the sound that will be played
                newsound = pygame.mixer.Sound(ihatepygame)
                #plays the temp variable
                pygame.mixer.Sound.play(newsound)
                #updates the videofeed to create less lag between inputs
                self.videoFeed.update()
                self.gui.updateImg(self.videoFeed.img)
                #finds the length of the audio file in ms and converts to seconds to allow enough time
                #for the file to finish playing
                pygame.time.wait(int(newsound.get_length()*1000))
                #updates the videofeed to create less lag between inputs
                self.videoFeed.update()
                self.gui.updateImg(self.videoFeed.img)

                
        
        except KeyError:
            print("hit a key error")

    #Set input timer end time by adding a delay to the current time reading
    def startInputTimer(self, delayTime = 1.0):
        self.delayTimerEndTime = monotonic() + delayTime

    #check if current time is greater than the generated time by startInpuDelayTimer: return False if greater
    def inputTimer(self):
        if monotonic() > self.delayTimerEndTime or self.delayTimerEndTime == 0:
        ###################################
        #    LED with check for toggle    #
        ###################################

            return False

        else:

            return True
            
        
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

# Main #
#initialize variables
window = None
detector = TouchDetector()
#display home window
detector.gui.home()
#operational loop
while True:
    
# Break loop when window is closed
    event, values = window.read(timeout=10)
    #updates the current image
    detector.videoFeed.update()
    #updates the video feed on the GUI with the most recent frame
    detector.gui.updateImg(detector.videoFeed.img)

    # checks to see if the checkbox has been pressed, if it is unchecked the LED is off.
    if values["led"]:
        #updates the visibility of the LED based on the boolean value output by the input timer
        window['LED'].update(visible=detector.inputTimer())
        #for the physical LED 
        #GPIO.output(LED_PIN,detector.inputTimer())
    if not values["led"]:
        window['LED'].update(visible=False)

    if not detector.inputTimer():
        detector.detectLetter()
    
    if event == 'exit' or event == sg.WIN_CLOSED:
        break

    #When Calibrate button is pressed
    if event == "calibrate":
        detector.calibrate()
        
    #When The ASL Chart button is pressed
    if event == "asl":
        sg.popup_no_buttons( title="ASL Chart",image="sign.png")

    
            
    

    #When Clear Input button is pressed
    if event == "clear":
        detector.outputStr = ""
        

        

    #Checks to see if the checkbox is currently true
    if values["speaker"] == True:
        print("Speaker On")
    

    #When Okay button is pressed in popup
    if event == "ok":
        window.close()
        detector.gui.home()
    
    window['output'].update('')
    print(detector.outputStr)
