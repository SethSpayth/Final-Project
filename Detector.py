#Import modules and libraries
from cvzone.HandTrackingModule import HandDetector
import cv2
from time import monotonic
import PySimpleGUI as sg

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

class TouchDetector():
    def __init__(self, gui = GUI(), videoFeed = Video()):
        self.videoFeed = videoFeed
        self.gui = gui
        self.DEBUG = True
        self.draw = False
        self.handDetector = HandDetector(detectionCon = 0.8, maxHands = 1)


    def calibrate(self):
        calibrationMatrix = []
        sum = 0

        #########################################################
        #   INSTRUCTION POP-UP & VIDEO FEED HOLDING PATTERN     #
        #########################################################

        while len(calibrationMatrix) < 20:
            self.videoFeed.update()
            self.findHands()
            if self.hand:
                m48, _ = self.handDetector.findDistance(self.hand['lmList'][4], self.hand["lmList"][8]),
                m412, _ = self.handDetector.findDistance(self.hand['lmList'][4], self.hand["lmList"][12]),
                m812, _ = self.handDetector.findDistance(self.hand['lmList'][8], self.hand["lmList"][12]),
                calibrationMatrix.append([m48, m412, m812])
            
            ##############################
            #    Display frame on GUI    #
            #    Possible progress bar   #
            ##############################

        #Average the measurements
        for measurementSet in calibrationMatrix:
            sum += max(measurementSet)
        average = sum / len(calibrationMatrix)
        
        #Set touch trheshold to average measurement
        self.TouchThreshold = average

        #####################
        #    Done Pop-up    #
        #####################

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

        primaryMeasurementPairs = [[(8,12,0x80), (12,16,0x40), (16,20,0x20)]]
        
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
            self.hand["lmList"][21] = self.hand["center"]

            #reset measurement base
            measurements = 0x00
            
            #for the point pairs see if they are touching and change the corresponding bit appropriately
            for pair in primaryMeasurementPairs:
                x, y, bit = pair
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
            0x66 : "Z"
        }
        try:

            detectedLetter = translationDictionary[self.getMeasurements()]
            
            if detectedLetter == "I":
                horizontalThreshold, _, _ = self.hand["lmList"][16]
                initialX, _, _ = self.hand["lmList"][20]
                for _ in range(11):
                    self.videoFeed.update()
                    self.findHands()
                    if self.hand:
                        termX, _, _ = self.hand["lmList"][16]
                        if termX < horizontalThreshold:
                            detectedLetter = "J"
                            break
            
            if detectedLetter == "Z":
                _, verticalThreshold, _ = self.hand["lmList"][0]
                _, initalY, _ = self.hand["lmList"][8]
                for _ in range(11):
                    self.videoFeed.update()
                    self.findHands()
                    if self.hand:
                        _, termY, _ = self.hand["lmList"][8]
                        if termY < horizontalThreshold:
                            detectedLetter = "Z"
                            break
                raise KeyError


            #####################################################################################
            #    Concatinate detectedLetter to output string    #
            #####################################################################################
            
            self.startInputTimer(1)

        except KeyError:
            pass

    #Set input timer end time by adding a delay to the current time reading
    def startInputTimer(self, delayTime = 1.0):
        self.delayTimerEndTime = monotonic() + delayTime
        
        ###################################
        #    LED with check for toggle    #
        ###################################

    #check if current time is greater than the generated time by startInpuDelayTimer: return False if greater
    def inputTimer(self):
        if monotonic() > self.delayTimerEndTime:
            return False
        
        ###################################
        #    LED with check for toggle    #
        ###################################

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