#Import modules and libraries
from cvzone.HandTrackingModule import HandDetector
import cv2 
from keyboard import is_pressed as pressed
import numpy as np
from time import monotonic
import abc

#Video Class that initializes a OpenCV video feed and updates it
class Video():
    def __init__(self, name = "Image"):
        #connect to default camera
        self.capture = cv2.VideoCapture(0)
        #capture name (Deault = "Image")
        self.name = name
        #Grab frame
        self.update()

    #function that updates frame from video feed and displays
    def update(self):
        #Grab frame
        self.success, self.img = self.capture.read()
        
        #display frame
        cv2.imshow(self.name, self.img)
        cv2.waitKey(1)      

#American Sign Language Detector Superclass
#Includes functions to calibrate, get measurements, and calculate other values based on video feed 
class ASLDetector(metaclass = abc.ABCMeta):
    
    #Initilization, passes a default generic videofeed
    #Initializes other varibles
    def __init__(self, videoFeed = Video()):
        self.handDetector = HandDetector(detectionCon=0.8, maxHands=1)
        self.measurements = []
        self.delayTimerEndTime = 0
        self.DEBUG = True
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
        self.distanceFromCamera = 1
        self.width = 2
        self.focalLength = 1
    
    #Function that detects the right hand from video feed
    def findHands(self, d = False):
        self.hands = self.handDetector.findHands(self.videoFeed.img, draw=d) 
        if self.hands:
            if self.hands[0]["type"] == "Right":
                self.hand = self.hands[0]
            else:
                self.hand = None
        else:
            self.hand = None

    #Function to calibrate the focal length of the camera in order to find distance between hand and camera
    def calibrate(self):
        
        #initialize calibration list and actual value
        calibrationMatrix = []
        actual = 0
        
        #print instructions for user
        print("Hold your right hand near your chest, palm facing the camera with fingers outstreched and touching")
        print("Please hold position for approximately 1 second")
        print("Press space when ready")
        
        #Wait for user to press Space
        while not pressed("space"):
            self.videoFeed.update()
        
        #Print hold and dots while 20 measurements are being taken
        #measurement is between base of hand and base of thumb
        print("Hold")
        while len(calibrationMatrix) < 20:
            self.videoFeed.update
            #calibrationMatrix.append((self.handDetector.findDistance(self.hand["lmList"][0], self.hand["lmList"][1]), self.handDetector.findDistance(self.hand["lmList"][0],self.hand["center"])))
            self.findHands()
            calibrationMatrix.append(self.handDetector.findDistance(self.hand["lmList"][0], self.hand["lmList"][1]))
            print(".")
        
        #Notify user when done
        print ("DONE")
        
        #Average the measurements
        for horizontalMeasurement, info in calibrationMatrix:
            horizontalAverage = actual
            actual = ((horizontalAverage + horizontalMeasurement)/2)
        
        #calculate focal length
        self.focalLength = (actual * self.distanceFromCamera)/self.width

    #Use distance formula (D = (known width * focal length) / measured width)
    def findDistanceFromCamera(self):
        measurement, info = self.handDetector.findDistance(self.hand["lmList"][0], self.hand["lmList"][1])
        self.distanceFromCamera = (self.width * self.focalLength)/measurement

    #Get measurement set defined by measurement pairs in __init__ and angle of hand
    def getMeasurements(self):
        self.findHands()
        self.measurements = []
        if self.hand:
            for index, pair in enumerate(self.measurementPairs):
                x, y = pair
                measurement, info = self.handDetector.findDistance(self.hand["lmList"][x], self.hand["lmList"][y])
                self.findDistanceFromCamera()
                self.measurements.append(measurement * self.distanceFromCamera)
        
            self.measurements.append(self.findAngle())
            return self.measurements
    
    #find angle between vertical line and line drawn between base of hand and base of middle finger  
    def findAngle(self):
        #unpack base and middle finger landmarks
        baseX, baseY, z = self.hand['lmList'][0] #base of hand coodinates
        middleFingerX, middleFingerY, z = self.hand["lmList"][9] #base of middle finger coodinates
        
        #define vertical line
        vertical = (baseX, 0)
        
        #create vectors
        baseToPalm = np.array([middleFingerX, middleFingerY]) - np.array([baseX, baseY])
        baseToTop = np.array(vertical) - np.array([baseX, baseY])
        
        #calculate angle
        self.angle = np.degrees(np.math.atan2(np.linalg.det([baseToPalm, baseToTop]), np.dot(baseToPalm, baseToTop)))
        return self.angle
    
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

#ASL detector that uses a dictionary of measurement ranges to detect letter
#Reads measurements and checks if each value falls between range for each measurement range for each letter
class ComparisonDetector(ASLDetector):
    def __init__(self):
        super().__init__()
        self.dict = {
                    'A' : [(55, 75), (25, 35), (25, 35), (25, 35), (145, 175), (80, 110), (60, 95), (45, 80), (55, 90), (70, 95), (90, 125), (110, 150), (-30, 0)],
                    'B' : [(140.49590678136454, 158.41392898106068), (27.459979796697375, 34.330647826605585), (26.712819265785267, 33.66396984271992), (45.037897769143, 52.27637146484479),     (105.57201278508214, 129.93429826622383), (246.70498838089765, 278.84575516090615),     (258.40287297572024, 291.11901498880013),   (243.34928345602788, 273.54779435640864),   (208.4795386478016, 237.6743025706103),   (150.95927921621794, 166.9407092058007), (135.54151647939105, 151.4817608986128),  (99.16170308299697, 117.6448584709604),    (-28.49563861824498, -25.2531633945739)],
                    #(113.97473506041842, 177.96601033084832), (27.889053614815033, 41.95234533433878),  (26.068568299222463, 39.58749863906848), (40.888851534509314, 61.377811084343875), (89.71453733018322, 132.99196756224214),  (231.54508548197862, 301.894868341409),       (241.1031322168373, 319.4005989038317),     (223.38366979491545, 298.8345704774566),    (192.43571415485744, 255.65642896223258), (124.2000186544614, 191.68105831473957), (113.07620128886875, 171.30509753013595), (90.54399749773991, 133.73173089274007),   (-23.54003926951431, -16.871432384327164)
                    #(140.91596051772692, 165.75388617941587), (27.20847066520718, 34.58550106717811),   (24.415168458761528, 29.77738922204909), (40.69130693545432, 48.55987295607947),   (93.60516032542479, 114.88422308785576),  (240.32959607755893, 265.05049312050585),     (253.15313966616742, 279.0033601885973),    (239.85313035813468, 266.84275973039297),   (204.17248362582527, 227.90963743227942), (153.42846021324837, 176.8211256937009), (140.87935317517434, 163.71626863754457), (108.14340788750837, 132.95303416540554),  (-25.91888557104643, -21.982438203118654)
                    #(140.49590678136454, 158.41392898106068), (27.459979796697375, 34.330647826605585), (26.712819265785267, 33.66396984271992), (45.037897769143, 52.27637146484479),     (105.57201278508214, 129.93429826622383), (246.70498838089765, 278.84575516090615),     (258.40287297572024, 291.11901498880013),   (243.34928345602788, 273.54779435640864),   (208.4795386478016, 237.6743025706103),   (150.95927921621794, 166.9407092058007), (135.54151647939105, 151.4817608986128),  (99.16170308299697, 117.6448584709604),    (-28.49563861824498, -25.2531633945739)
                    'C' : [(72.09589722802666, 90.83231502381359), (2.134730949934882, 10.614191029338972), (5.336827374837205, 13.509659616579015), (16.309610606057444, 25.734929537584943), (133.229037918512, 155.50786038540602), (168.98954197488604, 194.90165627525548), (165.9480374850507, 187.45681843737765), (161.47618294658386, 180.80082449651565), (144.1632181174364, 162.2133040224067), (72.26485852754514, 89.15799802315983), (73.69918949011071, 93.17387367403656), (72.9216921442003, 97.47170534838693), (-26.810954300348666, -21.80140948635181)],
                    'D' : [(132.9483073737894, 153.545908818391), (108.92920557642321, 127.11309081793286), (29.800515224162467, 37.6536371093785), (18.936747768493014, 24.01974656256381), (121.80430668133631, 133.14244658122328), (259.7967262069281, 285.5516342096089), (145.03028677409523, 159.59065566176682), (124.7520079346436, 139.11727377735178), (119.72230797385873, 134.9533989693316), (24.357310840393147, 32.950552846270625), (32.17562962787684, 40.054047210464404), (52.38677559792395, 60.422903292831236), (-26.565051177077983, -23.96248897457819)],
                    'E' : [(38.53140940468552, 52.50324092191747), (24.05382534665479, 30.847968458350728), (25.072217643083466, 29.860435686851865), (20.39760309879786, 25.919354806725963), (105.05260671668614, 117.31965433557608), (128.3236620475029, 146.69179925702528), (115.4425949715499, 131.20513395218467), (109.25869594759025, 121.8053792708288), (112.74623994836342, 125.36274320516377), (13.346871842457526, 23.051504355059627), (9.694956383999179, 19.896341817808594), (31.85399236954151, 42.61934602037586), (-29.156917882313007, -21.113500078365238)],
                    'F' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'G' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'H' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'I' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'K' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'L' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'M' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'N' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'O' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'P' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'Q' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'R' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'S' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'T' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'U' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'V' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'W' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'X' : [(0,0) for _ in range(len(self.measurementPairs)+1)],
                    'Y' : [(0,0) for _ in range(len(self.measurementPairs)+1)]                  
                        }
    
    def detectLetter(self):
        try:
            for letter in self.dict:
                for index, val in enumerate(self.measurements):
                    min, max = self.dict[letter][index]
                    
                    if val < (min) or val > (max):
                        break
                    
                    elif index == len(self.measurements) - 1:
                        print (letter)
                        self.startInputTimer(4)
                        raise StopIteration
        
        except StopIteration:
                return letter

#ASL detector that "plots" the letters on an n-dimentional graph then takes the finds the point nearest to the measured values and returns that letter if 
#the distance between the points is less than a set margin
class PlotDetector(ASLDetector):
    def __init__(self):
        super().__init__()
        self.dict = {
                    "A" : [55, 20, 20, 20, 125, 70, 60, 50, 55, 65, 85, 100, -30]
                    }
    
    def detectLetter(self):
        margin = 20
        mostLikelyLetter = None
        distances = [0 for _ in self.dict]
        
        for i, letter in enumerate(self.dict):
            for index, measurement in enumerate(self.measurements):
                
                distances[i] += (self.dict[letter][index] - measurement) ** 2
                
            distances[i] = distances[i] ** 0.5
            
        if self.DEBUG:
            print(distances[i])
            
        if distances[i] < margin:
            margin = distances[i]
            mostLikelyLetter = letter
            
        if mostLikelyLetter:
            print(f"{mostLikelyLetter}\tmargin{margin}")
            self.startInputTimer(4)
            return mostLikelyLetter

def main():
    measurementMatrix = []
    detector = ComparisonDetector()
    detector.calibrate()
    while not pressed("tab"):
        detector.videoFeed.update()
        detector.getMeasurements()
        detector.detectLetter()
        if pressed("R"):
            measurementMatrix.append(detector.measurements)
    list = detector.createMeasurementRange(measurementMatrix)
    print (list)
    
if __name__ == "__main__":
    main()
