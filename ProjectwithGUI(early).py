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
    def __init__(self, videoFeed = Video(), D = True):
        self.handDetector = HandDetector(detectionCon=0.8, maxHands=1)
        self.measurements = []
        self.delayTimerEndTime = 0
        self.DEBUG = True
        self.D = D 
        self.videoFeed = videoFeed

    ##Accessor for the Draw variable findHands
    @property
    def D(self):
        return self._D
    ## mutator for the Draw variable in findHands
    @D.setter
    def D(self, value):
        self._D = value       
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
        self.width = 46
        self.focalLength = 1
    
    #Function that detects the right hand from video feed
    def findHands(self):
        self.hands = self.handDetector.findHands(self.videoFeed.img, draw=self.D) 
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
            if self.hands:
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
        self.dictionary = {
                            "A" : [(2.750831359922252, 3.374109569147894), (0.9402905622537188, 1.059753236879663), (0.8834522085987723, 1.0285714285714285), (0.8957803342713111, 1.2356872892941309), (6.306795835224928, 6.9487132178998), (3.4622296335824405, 3.7090882232776696), (2.909772279125899, 3.2261022452827706), (2.6112184011418, 2.916767680740102), (2.780459717361562, 3.1144823004794877), (3.380359340113515, 4.043037700313199), (4.1463468489975845, 4.799439069788822), (4.90608236874725, 5.667722375264771), (-24.82934658814961, -21.801409486351794)],
                            "B" : [(6.155095949203109, 7.328531281032627), (1.2376781220325266, 1.3999863815216702), (0.9855645994138762, 1.3529904132350055), (1.7200577682492673, 2.072397037960703), (3.4628310845191383, 4.4206868998048225), (10.377891738820326, 10.91609388998931), (10.993049338466639, 11.560426893994933), (10.45707026233149, 10.844423694330393), (9.046025815699723, 9.364354457928606), (6.746549011671002, 7.740852154589179), (6.282232970158782, 7.176848243837368), (5.015739172872024, 5.795563722010621), (-25.057615418303023, -22.40983737149326)],
                            "C" : [(1.5842226447966001, 1.7236874037643697), (0.04426989514189718, 0.1397239298497867), (0.08944271909999159, 0.23287321641631115), (0.21504133222818814, 0.31243220522753506), (5.709615225779896, 6.063261692427309), (6.3866899812968425, 6.718542018305942), (6.3505315027536815, 6.629762984106861), (6.384695075740977, 6.668362019500844), (6.227123739231001, 6.5950275707676065), (1.4998192226567704, 1.711724276862369), (1.6466623048469606, 1.827851323939599), (1.7927941125612548, 1.987824209205935), (-10.95406264339832, -8.29714496983687)],
                            "D" : [(4.612134455148195, 5.499967014019042), (3.871643908731354, 4.440544168329524), (0.5010362705417046, 0.7410709337195484), (0.05022019609518254, 0.16238174542072156), (5.605643273926982, 6.1735895188431344), (7.7813747198756795, 9.01683637265658), (5.8904240454898185, 6.704774312724615), (5.512151796475301, 6.335560935366928), (5.417047012925858, 6.2595065581269), (0.7539660007523259, 1.2156097261742467), (0.3672257776684969, 0.5547001962252291), (0.432113401436758, 0.5655853302081291), (-13.172553423326892, -10.813877513624961)],
                            "E" : [(2.7270722537041583, 3.1086814592263647), (0.9516985496416492, 1.0751744044572489), (1.0046047826039697, 1.2306595067591861), (0.7434938529705477, 0.968379747877652), (3.837467113364471, 4.323533977904104), (4.687462820365372, 5.225593723735645), (4.639960212031, 5.130657994086349), (4.373697608343627, 4.796696650319542), (4.178332387264271, 4.690842141876019), (1.9062649747205012, 2.1986416025049556), (0.8705491499914476, 1.0601886624558856), (0.19026059766179765, 0.4242640687119285), (-23.286064918223598, -22.051062719076743)],
                            "F" : [(1.096119857984697, 1.5485548907581468), (5.345023317961956, 6.123976668379611), (2.188864441117734, 2.535288643197521), (3.1616930816145046, 3.4996026615357256), (6.32016986598741, 6.793968218795563), (7.17873219541282, 7.896111605845334), (11.787395755895357, 13.110729336425326), (11.16106249786869, 12.289261896989823), (9.69497840557977, 10.809966559090395), (6.447570434680522, 7.66737425301016), (6.81034333496564, 8.008030579991969), (7.352995541823594, 8.60304370205286), (-22.203478532057378, -20.695450734063307)],
                            "G" : [(0.2226016031594911, 2.573617021672706), (3.3092107358323073, 4.412863443925456), (0.8719855623900316, 1.2287097405242393), (0.7853855195716564, 1.0622033054311992), (6.6951690298540605, 7.958061277698915), (6.9075793052400245, 8.71971820303471), (3.68656658340214, 4.407198228627241), (2.888231351862302, 3.4198091086525046), (2.227846076318723, 2.7805252659734956), (3.0688509500863925, 3.582056449118196), (3.77970931125314, 4.644229924899363), (4.402632120180741, 5.607670328617729), (0.0, 3.0127875041833416)],
                            "H" : [(2.7373333064450236, 3.0289802227137566), (0.8904724456548828, 1.4766637959093822), (4.968435399446322, 5.740465240420029), (0.8469010445797931, 0.9471318934179186), (5.944482317790608, 6.326880097228481), (8.571635851835435, 9.227081524363436), (8.532429486446196, 9.568224985232618), (3.552960654407069, 3.9782496890259784), (2.9921465833863468, 3.449211333555746), (3.1544552993019437, 3.764439362969222), (2.2312202825011664, 2.671417336964648), (3.1086483023198235, 3.5370309915653997), (-27.235146699302508, -23.824260582895423)],
                            "I" : [(1.819707198923356, 2.036420523125945), (0.8035073760083702, 0.9373539772319093), (0.8171215305645165, 0.9464224429830758), (4.997847456311233, 5.505242256948581), (5.7507040753542835, 6.0820599299458555), (3.8298417194072263, 4.092937423432803), (3.416920333217155, 3.660834522579435), (3.560029939478696, 3.8435204660115407), (8.34702844056862, 8.914024290960231), (2.436569783507045, 2.727510997102714), (2.930569107548506, 3.254451258116027), (5.850848201516421, 6.4468901931391045), (-19.069193537348134, -17.35402463626132)],
                            "K" : [(3.2086478421430447, 3.6210504617326427), (3.312449364879408, 3.696809173608471), (6.926768925709694, 7.490187218241967), (0.7427813527082076, 0.9407358957534817), (6.345840749579254, 6.758378183830472), (9.32041418891593, 9.950082882950944), (9.274766395777226, 9.968914339687808), (2.328668313814877, 2.5011874574525614), (2.7687362429711073, 3.0063054873581803), (3.305156519251315, 3.6760731104690385), (4.027231648173253, 4.318060114626355), (3.8876237022067492, 4.259234536647903), (-7.8290765100596, -6.340191745909913)],
                            "L" : [(4.9285438794279255, 5.819808635639513), (5.859768054547856, 6.682369892395984), (1.0886183786309411, 1.3085899126818605), (0.8634241630548938, 1.0382970698498735), (6.654549622816689, 7.470126756296412), (9.018730943274445, 10.068132728120938), (3.079912528387662, 3.4288159712453528), (2.3851511514837305, 2.633305153012249), (2.4531601779361707, 2.792529245069733), (4.080897662984768, 4.705632213372258), (5.198381328916391, 6.007390682333636), (6.050921466592124, 7.005573552911912), (-7.125016348901791, -4.653351587268388)],
                            "M" : [(2.3459543783981163, 3.308300612274597), (0.7913560158938597, 0.9201590118038789), (0.76330957640656, 1.072723307343346), (1.0833009074265232, 1.6188929796740763), (4.953458603516551, 5.6815627816669725), (3.762614112191809, 4.278986752311713), (3.2539568672798427, 3.9679651059090806), (3.408313919460017, 3.8706719271097074), (2.3200591622629663, 3.1263907411179206), (1.9002354963620276, 2.767989289336639), (1.5827120317877195, 1.8825827065653247), (2.630608470515364, 2.8984021967486924), (-30.311213226681623, -27.75854060106003)],
                            "N" : [(2.8102064468598393, 3.185204882818395), (0.6859943405700353, 0.8851810795451521), (2.119886683523963, 2.434650290538338), (0.9611939478116446, 1.1606262623517603), (5.790876149197327, 6.4765689949189875), (4.308707727377952, 4.756000695914858), (3.9948597108770967, 4.368189680071263), (1.9061903482694524, 2.167883614009637), (2.367010715232907, 2.682041609175218), (2.3320283369598154, 2.7121042302074887), (3.793496000098871, 4.350544210467803), (3.717783991602952, 4.123861904360645), (-24.829346588149598, -20.924501744921187)],
                            "O" : [(0.5858287004510699, 0.7489417237567663), (0.04865042554105199, 0.15670016490742283), (0.09747403576571587, 0.2103378946562697), (0.39223227027636803, 0.5186863531990444), (5.957057407083584, 6.31673222883068), (6.161531458496346, 6.595280345572496), (6.125712636164025, 6.6414835570561115), (6.119347868695559, 6.604743864029396), (6.181959148293912, 6.534030829506155), (0.6852321848892694, 0.8579280908893565), (0.7705999143709422, 0.978188525734264), (1.227881227029841, 1.42583130974806), (-4.351077951573884, -0.6095065766751957)],
                            "P" : [(1.6228960012524414, 2.091202668047353), (2.696145569318212, 3.0794969302744977), (2.5325519891475325, 3.007556598088975), (0.28884877378921775, 0.5154056598853398), (5.188648469173862, 5.579357319674651), (6.745952081194012, 7.316703737685828), (4.523345287635117, 5.549274660961801), (2.09780069461296, 2.6011556822333284), (1.7978447322786126, 2.379769923229911), (0.9913251372940636, 1.8451919285290364), (2.9331500244361726, 3.178470212109281), (3.243322743669189, 3.5129918278470496), (-20.61818812310451, -14.420773127510985)],
                            "Q" : [(1.2740104217036075, 1.9300054755483695), (3.3864945086897977, 3.8799075886391323), (0.6808318626326626, 1.035642411765431), (0.43701422443753785, 0.6084679506313999), (5.656461544202052, 6.820912018515647), (6.87724921976828, 8.322328189684379), (3.59047652250296, 4.50502698535772), (2.9773922927395486, 3.687817782917154), (2.6981969105196533, 3.539327824345996), (2.2586115779272338, 2.5606257038087863), (2.7333691524127723, 3.1995630163555866), (2.967178223896534, 3.392259680887396), (-49.69868051729944, -43.636072468397096)],
                            "R" : [(4.278704601268206, 4.730474911644619), (0.40662244453625024, 0.8355588111087436), (6.6211781428987395, 7.076722405181653), (0.9489863986806419, 1.1107224192064469), (5.516230597065356, 5.930860908090082), (9.864684485577833, 10.55972293219572), (10.271048418349642, 10.908252502242929), (3.4595091411376933, 3.962449082919689), (2.8216740061775942, 3.4008953724789546), (4.6746978511985136, 5.232649695648491), (1.7992968589213507, 2.1545972589260622), (2.687198417085262, 3.0366845880330136), (-22.714412353167244, -17.31893843151474)],
                            "S" : [(1.1267902987701157, 1.4142135623730951), (1.1156807257981907, 1.3275451923093853), (0.9665337740610512, 1.2509508578599273), (0.8219470612283083, 1.2602058609136797), (4.658644684565427, 5.292608744000748), (3.9250402028589986, 4.50906945134727), (3.352690832493762, 3.939543120718442), (2.9383096103604593, 3.4307078129098922), (2.897592035459857, 3.492151478847891), (1.212678125181665, 1.4487960177299284), (2.0907710370606734, 2.37876787126568), (2.8701260882365847, 3.4375260531163483), (-25.057615418303012, -21.801409486351805)],
                            "T" : [(2.835083822879338, 3.2310732001761067), (1.2280111196910335, 1.636474497353321), (1.245336791236991, 1.422988498042119), (1.0492131605931765, 1.3488929516911086), (6.84003296010931, 7.44672132797954), (4.263515096209906, 4.821193051416256), (2.8249757175731705, 3.241589587817721), (2.3223854306061575, 2.6551631852167112), (2.817743697996077, 3.1853322375247815), (3.7670129317402417, 4.461453161846787), (4.5303458952109255, 5.039264913791424), (4.862675944593684, 5.333739821907784), (-19.85521436932106, -17.265809495014572)],
                            "U" : [(5.5466515088194255, 6.574077662290475), (0.9780995349968278, 1.2538956636260115), (6.840102602308558, 7.8723926778323525), (0.5528878686860184, 1.0040380896903927), (5.437092584173302, 5.772368297015105), (10.454891344671688, 11.42940129109274), (10.9191316881474, 11.910455891256733), (4.0, 4.401285756058484), (3.6271296315629318, 4.2016929883757355), (5.678828882166127, 6.724298102486108), (1.5457566828893403, 1.7246622060423555), (1.5264062771816589, 2.001155735433064), (-19.476575498932, -17.05696557661706)],
                            "V" : [(7.082050579776155, 7.535010979778348), (4.989107305905994, 5.285922451516918), (7.469454204558031, 8.092821330921073), (0.5190665196422048, 0.9218139360209776), (5.472872336005803, 5.92959066065704), (11.419463432128508, 11.951472062030234), (11.421822581710982, 11.96381225330104), (3.7057580178752407, 4.0881323772245555), (3.486168079087362, 3.9963714111627), (5.747358032377277, 6.2607002614647405), (1.7335458883889572, 2.0782615293691884), (1.6211043379780954, 2.204094164976375), (-15.689992929083877, -14.489762593884448)],
                            "W" : [(8.050352323421288, 8.850362278847319), (3.0948314415127927, 3.413627065297649), (2.3067782467506883, 2.7194163830114326), (6.680311424360935, 7.498894701931954), (5.599172956603223, 6.061857305285015), (12.219149110896579, 13.2362736067234), (12.257546495281813, 13.390681268239724), (11.617826530942212, 12.61761641641678), (4.876455322645894, 5.334412671440297), (7.114755813362005, 7.952111308989395), (6.093110305054171, 6.717489742084707), (0.5755895140396069, 0.9790552705531631), (-16.525796389925638, -15.046391832198493)],
                            "X" : [(4.368171797845333, 4.685714285714286), (4.285342675879139, 4.600438876334472), (1.0671121420447909, 1.2737601417048436), (0.951228780813395, 1.0845676795304473), (3.9278169082506738, 4.204176737580894), (8.206158691033687, 8.775662077470804), (3.9248725173509325, 4.247063811671325), (2.824520840969766, 3.1089075448538157), (2.123341786715866, 2.345642943059862), (0.4027983240526136, 0.6368800708524218), (1.2744714420682004, 1.4996855016214516), (2.2515248525942835, 2.5407751221878847), (0.0, 2.43664824681013)],
                            "Y" : [(3.092225538496406, 3.5703091516024954), (0.7515825187151051, 1.0806864539165983), (0.7446737923427352, 0.9199353193217104), (4.051568610748695, 4.784617224342097), (6.288382444784739, 6.9970477835977665), (3.806420266394349, 4.168357783536611), (3.4292761798372773, 3.6309049938521065), (3.3841258387222677, 3.624070128670341), (7.29563654612511, 8.188280276364354), (3.840199652826229, 4.588883441499623), (4.58856388507987, 5.357006688548841), (6.619953014326383, 7.663156460763982), (-13.324531261890789, -11.592175410291063)],           
                        }
    
    def detectLetter(self):
        try:
            for letter in self.dictionary:
                for index, val in enumerate(self.measurements):
                    min, max = self.dictionary[letter][index]
                    
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
        self.dictionary = {
                    "A" : [55, 20, 20, 20, 125, 70, 60, 50, 55, 65, 85, 100, -30]
                    }
    
    def detectLetter(self):
        margin = 20
        mostLikelyLetter = None
        distances = [0 for _ in self.dictionary]
        
        for i, letter in enumerate(self.dictionary):
            for index, measurement in enumerate(self.measurements):
                
                distances[i] += (self.dictionary[letter][index] - measurement) ** 2
                
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
            print(ASLDetector.D)
            if(ASLDetector.D == True ):
                ASLDetector.D = False
            else:
                ASLDetector.D=True
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
