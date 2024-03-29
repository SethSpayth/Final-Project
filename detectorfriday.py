#Import modules and libraries
from cvzone.HandTrackingModule import HandDetector
import cv2
from time import monotonic
import PySimpleGUI as sg
from math import hypot
#import RPi.GPIO as GPIO
import os
from playsound import playsound
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
        #capture name default = Image
        self.name = name

    #updates img with a new frame
    def update(self):
        _, self.img = self.capture.read()

class GUI():
    def __init__(self):
        pass
    def home():
        global window
        # changing the color theme of GUI    
        sg.theme('LightBlue')   
        # the header text within the GUI
        cameraRow   = [[sg.Push(), sg.Image(filename='', key='image', size=(10,10)),sg.Push()]]
        buttonRow1  = [[sg.Push(),sg.Button("Calibrate", key="calibrate", size = 20),sg.Button("Show Lines", key="showlines", size = 20),sg.Checkbox("Toggle LED Light",default=True, key="led"),sg.Push()]]
        buttonRow2  = [[sg.Push(),sg.Button("Clear Input", key="clear", size = 20),sg.Button("Exit", key = "exit", size = 20),sg.Checkbox("Toggle Speaker",default=True, key="speaker"),sg.Push(),]]
        welcomeText = [[sg.Push(),sg.Text("ASL Detector Alpha", size =30,  font='Helvetica 20'),sg.Push()]]
        detectedletterString = [[sg.Output(size=(80,20), key="output")]]
        # the formatting for the layout of the GUI 
        layout =  [[welcomeText],
                   [cameraRow],
                   [buttonRow1],
                   [buttonRow2],
                   [detectedletterString]]

        window = sg.Window('ASL Detector', layout, size = (w,h))

    def updateImg(self, img):
        imgbytes = cv2.imencode('.png', img)[1].tobytes()  # ditto
        window['image'].update(data=imgbytes)
        cv2.waitKey(1)



class TouchDetector():
    def __init__(self, gui = GUI(), videoFeed = Video()):
        self.videoFeed = videoFeed
        self.gui = gui
        self.DEBUG = True
        self.draw = False
        self.handDetector = HandDetector(detectionCon = 0.8, maxHands = 1)
        self.outputStr = ""
        self.delayTimerEndTime = 0
        self.touchThreshold = 20

    def findDistance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        length = hypot(x2 - x1, y2 - y1)
        return length

    def calibrate(self):
        calibrationMatrix = []
        sum = 0

        #########################################################
        #   INSTRUCTION POP-UP & VIDEO FEED HOLDING PATTERN     #
        #########################################################
        
        #global window
        #instructionText = [[sg.Text(text="Connect your thumb to your pointer finger combining them to form an ok symbol ",font="bold")]]
        #okButton        = [[sg.Button("Okay", key="ok", size = 20)]]
        #popuplayout     = [[instructionText, okButton]]
        #window          = sg.Window('Calibration Instructions', popuplayout, size = (w,h))

        instrctionPopUP = sg.popup_ok("Connect your thumb to your pointer finger combining them to form an ok symbol", keep_on_top=True)
        
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
                sg.one_line_progress_meter("Calibration Completion", len(calibrationMatrix), 20)
            
            ##############################
            #    Display frame on GUI    #
            #    Possible progress bar   #
            ##############################

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
            self.startInputTimer(1)
            self.outputStr+= detectedLetters
            window['output'].update('')
            print(self.outputStr)
            if not window["speaker"]:
                playsound(f"{translationDictionary[self.getMeasurements()]}.wav")
        
        except KeyError:
            print("hit a key error")

        except KeyboardInterrupt:
            print("Type Error")

    #Set input timer end time by adding a delay to the current time reading
    def startInputTimer(self, delayTime = 1.0):
        self.delayTimerEndTime = monotonic() + delayTime
        
        ###################################
        #    LED with check for toggle    #
        ###################################
        if window["led"]:
            #GPIO.output(LED_PIN, GPIO.HIGH)
            print("LED ON ")
        
    
    #check if current time is greater than the generated time by startInpuDelayTimer: return False if greater
    def inputTimer(self):
        if monotonic() > self.delayTimerEndTime or self.delayTimerEndTime == 0:
        ###################################
        #    LED with check for toggle    #
        ###################################
            if not window["led"]:
                #GPIO.output(LED_PIN, GPIO.LOW)
                print("Lool")
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
GUI.home()
#operational loop
while True:
    
# Break loop when window is closed
    event, values = window.read(timeout=10)
    detector.videoFeed.update()
    detector.gui.updateImg(detector.videoFeed.img)

    if not detector.inputTimer():
        detector.detectLetter()
    
    if event == 'exit' or event == sg.WIN_CLOSED:
        break



    #When Calibrate button is pressed
    if event == "calibrate":
        detector.calibrate()
        


    #When Showlines button is pressed
    if event == "showlines":
        detector.draw = True

    #When Toggle LED Checkbox is pressed
    if window["led"]:
        #print("LED on ")
        pass

    #When Clear Input button is pressed
    if event == "clear":
        detector.outputStr = ""
        

        

    #When Speaker Checkbox is pressed
    if window["speaker"]:
        print("Speaker On")
        
    
    #When Okay button is pressed in popup
    if event == "ok":
        window.close()
        detector.gui.home()
    
    window['output'].update('')
    print(detector.outputStr)