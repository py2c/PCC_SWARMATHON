import roslib; roslib.load_manifest('mobility')
import rospy
# not using numpy.random because not thread safe
from random import random
from sys import argv
import socket
import tf

import numpy as np # for random number generation -- math packages
# http://docs.scipy.org/doc/numpy-1.10.0/reference/routines.math.html --- Use this link for numpy math function details

# numpy requires pip install numpy


global DEBUG_MODE
DEBUG_MODE = False


##############################################################

####################### VECTOR ###############################

class Vector:

    def __init__(self, x = None, y = None):
        if x is not None and y is None:
            try:
                self.x = x.x
                self.y = x.y
                    
            except AttributeError:
                try:
                    self.x = x[0]
                    self.y = x[1]
                         
                except AttributeError:
                    raise NotImplementedError
                        
        elif not (x is None and y is None):
            self.x = x
            self.y = y
        else:
            self.x = 0
            self.y = 0
    
    #sets a vector to another vector
    #or sets a vector to x & y
    def setVector(self, x, y = None):
        if y == None:
            try:
                self.x = x.x
                self.y = x.y
                    
            except AttributeError:
                try:
                    self.x = x[0]
                    self.y = x[1]
                
                except AttributeError:
                    raise NotImplementedError
        else:
            self.x = x
            self.y = y
    
    '''
    ACCESSORS AND MUTATOR
    '''
    #gets and sets the vector array
    def vector(self, x = None, y = None):
        if x is not None:
            if y is None:
                try:
                    self.x = x.x
                    self.y = x.y
                except AttributeError:
                    try:                            
                        self.x = x[0]
                        self.y = x[1]        
                    except AttributeError:
                        raise NotImplementedError
            else:
                self.x = x
                self.y = y
                
        return np.array([self.x, self.y])

    '''
    MEMBER FUNCTIONS
    '''
    #returns a unit vector
    #does not change member variable
    def unit(self):
        if(self.magnitude() == 0):
            return self 
        return Vector(np.array([self.x, self.y]) / self.magnitude())
    
    #returns the magnitude of the vector
    def magnitude(self):
        return np.linalg.norm(np.array([self.x, self.y]))
    
    #returns the dot product of vector and other vector
    #does not change member variable
    def dot(self, other):
        return np.dot(np.array([self.x, self.y]), np.array([other.x, other.y]))

    #returns the magnitude from one vector to the other vector
    def distanceFrom(self, other):
        return (self - other).magnitude()

    '''
    ALTERING MEMBER FUNCTIONS
    '''
    #adds vector to other vector
    def add(self, other):
        #does not change member variable
#         return Vector(np.add(self.vector, other.vector))
        self.x += other.x
        self.y += other.y
        
        return self

    #subtracts vector from other vector
    def subtract(self, other):
        #does not change member variable
#         return Vector(np.subtract(self.vector, other.vector))
        self.x -= other.y
        self.y -= other.y
        return self
    
    #multiplies vector by a scalar
    def timesScalar(self, scalar):
        #does not change member variable
#         return Vector(scalar * self.vector)
        self.y *= scalar
        return self
    
    #divides vector by a scalar
    def divide(self, scalar):
	if(scalar == 0):
	    return None
        self.x /= scalar
        self.y /= scalar
        return self
    
    '''
    OPERATORS
    '''
    #operator overloading for + 
    def __add__(self, other):
        return Vector(np.add(np.array([self.x, self.y]), np.array([other.x, other.y])))

    #operator overloading for -
    def __sub__(self, other):
        return Vector(np.subtract(np.array([self.x, self.y]), np.array([other.x, other.y])))

    # if operand is int type then it's scalar
    # if operand is a Vector type, then it's a dot product
    def __mul__(self, other): # giving me some bugs, don't use yet
        try:
            return np.dot(np.array([self.x, self.y]), np.array([other.x, other.y]))
        except AttributeError:
            return Vector(other * np.array([self.x, self.y]))
    #right hand Vector multiplication
    def __rmul__(self, other):
        try:
            return np.dot(np.array([self.x, self.y]), np.array([other.x, other.y]))
        except AttributeError:
            return Vector(other * np.array([self.x, self.y]))
    
    # operator overloading for / 
    def __truediv__(self, scalar):
        return Vector(np.divide([self.x, self.y], scalar))

    def __floordiv__(self, scalar):
        return Vector(np.divide([self.x, self.y], scalar))
    '''
    ANGLES
    '''
    #returns angle in radians
    def radians(self):
        return np.arctan2(self.y, self.x)
    
    #returns angle in degrees
    def degrees(self):
        return np.degrees(self.radians())
        
    #returns difference of angle between self and other in radians
    def radiansDifference(self, other):
        if self.__class__ == other.__class__:
            result = self.radians() - other.radians()
            if result > np.pi:
                result -= 2 * np.pi
            elif result < -np.pi:
                result += 2 * np.pi
            return result
            
    #returns difference of angle between self and other in degrees
    def degreesDifference(self, other):
        if self.__class__ == other.__class__:
            
            result = self.degrees() - other.degrees()
            if result > 180:
                result -= 360
            elif result < -180:
                result += 360
            return result
            
    def __str__(self):
        return np.array([self.x, self.y]).__str__()

#############################################################
    
#################### STACK CLASS#############################
class Stack ():
    
    def __init__(self):
        self.items = []
    def push(self, item):
        self.items.append(item)
    def pop(self):
        return self.items.pop()
       
    # peeks gives you the element at the top of the stack 
    def peek(self):
        if len(self.items):
            return self.items[-1]
        else:
            return None
    
    def isEmpty(self):
        return self.items == []
        
    # everything that is inside of the stack     
    def getElements(self):
        return self.items
        
    # checks if funct is in list
    def contains(self, funct):
        return funct in self.items
        
    # pops until funct in list is popped off
    # if funct not in list do nothing
    def popUntil(self, funct):
        if self.contains(funct):
            while self.peek() is not funct:
                self.pop()
            self.pop()
    

stateStack = Stack()
##############################################################

# Just import the messages, we don't actually have to subscribe/publish
# the ROS Message types
from std_msgs.msg import UInt8, String, Int16
from sensor_msgs.msg import Range, Joy, NavSatFix, Image #@DAVID WAS HERE
from geometry_msgs.msg import Pose2D, Point, Twist, Quaternion
from nav_msgs.msg import Odometry
from shared_messages.msg import TagsImage #@DAVID WAS HERE

# because we are assigning to these variables, we don't need to preface that
# they are global!

# rather than recalculate every time... why don't these things exist in python
# again?
global M_PI, M_PI_2, M_PI_4, M_2_PI
M_PI = np.math.pi
M_PI_2 = M_PI / 2.0
M_PI_3 = M_PI / 3.0
M_PI_4 = M_PI / 4.0
M_PI_8 = M_PI / 8.0
M_2_PI = M_PI * 2.0

global currentLocation, goalLocation, currentGPS, homeOffset
global msgVelocity 
#stateMachineState, 
global prev_state_machine, msgDebug, passedOnce
global targetDetected
home = NavSatFix(latitude = 0.0, longitude = 0.0)
homeList = []
currentGPS = NavSatFix()
currentLocation = Pose2D()
homeOffset = Pose2D()
goalLocation = Pose2D()
avoidGoalLocation = Pose2D()
pathLocation = []
prevTagLocationXY = []
status_publish_interval = 5
status_queue_limit = 10
mobilityLoopTimeStep = .750E8 # .0750 seconds in nanoseconds
msgVelocity = Twist()
msgDebug = String()
#stateMachineState = STATE_MACHINE.TRANSFORM
prev_state_machine = ''
currentMode = 0
targetDetected = Int16()
passedOnce = False
#stateStack = Stack()
rover_names_list = []
subRoverLocationsList = []

#beenTo variables
resolution = 20
x_max = 50 * resolution
y_max = 50 * resolution
offset = x_max // 2 # to make [0,0] the center of the 2D array
# Each beenToGrid cell contains [count, vector force]
beenToGrid = [[[0,Vector(0,0)] for x in range(x_max)] for y in range(y_max)]

# forces
currentForce = Vector()
baseForce = Vector()
obstacleForce = Vector()
forceList = []
repulsionFactor = 0
repulsionForceDic = {}
# non-global forces
# wandering force

# Running Average variables
movingLatList = []
currentLatAvg = 0
currentLatLen = 0
movingLongList = []
currentLongAvg = 0
currentLongLen = 0
avgLen = 15

# Average Osbstacle Detection variables
sonarLeftList = []
currentSonarLeftAvg = 0
currentSonarLeftLen = 0
sonarRightList = []
currentSonarRightAvg = 0
currentSonarRightLen = 0
sonarCenterList = []
currentSonarCenterAvg = 0
currentSonarCenterLen = 0

# unassigned globals
global pubDebug
global pubVelocity
global pubStatus
global pubState
global pubTargetsCollected
global pubGPSLocation
global pubName
global subGPSLocation
global subMode
global subTargets 
global subTargetsCollected
global subOdometry
global subObstacle
global subFix
global subJoystick
global subName
global timerStatus

# networking
global hostname
global publishedName

################################# David's Globals#################################### #@DAVID WAS HERE
global homeTagID
global currentlyHeldTag
global tagLocationList
global temporaryImageData
global targetsCollected
global targetSubscriber
global targetDropOffPublish
global targetPickupPublish  #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
#stateStack stuff
global holdingTag
holdingTag = False

targetsCollected = [0] * 586 # Updated based on new specifics. 2/26/16
temporaryImageData = None
homeTagID = 256
tagLocationList = []
#####################################################################################

def main(argv):
    global pubVelocity, pubStatus, pubState, pubDebug
    global pubTargetsCollected, pubStatusTimerCallback  
    global pubGPSLocation, subGPSLocation
    global publishedName, hostname 
    global home, stateStack
    global homeTagID
    global pubName, subName
    global currentLocation, goalLocation, targetDetected 
    global subMode, subTargets, subObstacle, subOdometry, subFix, subJoystick
    global subSonarLeft, subSonarRight, subSonarCenter, subTargetsCollected, mobilityStateMachine
    global targetSubscriber, targetDropOffPublish, targetPickupPublish #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!

    # set up name for use in the publishers
    hostname = socket.gethostname()
    publishedName = hostname if len(argv) < 2 else argv[1]
     
    # set up initial random direction and that we have not spotted a tag
    targetDetected.data = -1 # no tag seen
    
    # initialize the node
    rospy.init_node((publishedName + '_MOBILITY'), disable_signals=True)

    # confirm startup
    print('Node is started at: /'+publishedName)
    
    # set up subscribers to have callbacks when info received on these
    # topics
    subMode = rospy.Subscriber((publishedName + '/mode'), UInt8, 
            modeCallback, queue_size = 1)
    subObstacle = rospy.Subscriber((publishedName + '/obstacle'), UInt8, 
            obstacleCallback, queue_size = 10)

    subOdometry = rospy.Subscriber((publishedName + '/odom/ekf'), Odometry, 
            odometryCallback, queue_size = 10)
    subTargetsCollected = rospy.Subscriber(('targetsCollected'), Int16, # check if this is Int16 type and not TagsImage type 
            targetsCollectedCallback, queue_size = 10)
    subFix = rospy.Subscriber((publishedName + '/fix'), NavSatFix,
            updateGPSCallback, queue_size = 10)
    subGPSLocation = rospy.Subscriber('GPSLocation', NavSatFix,
            setHomeCallback, queue_size = 10)
    subJoystick = rospy.Subscriber((publishedName + '/joystick'), Twist, 
            joystickCallback, queue_size = 10)
    subName = rospy.Subscriber('rover_names', String,
            nameCallback, queue_size = 10)
    targetSubscriber = rospy.Subscriber((publishedName + "/targets"), TagsImage,
            targetCallback, queue_size = 10) #@DAVID WAS HERE
            
    # set up publishers to write data to these topics
    pubStatus = rospy.Publisher((publishedName + '/status'), String, 
            queue_size = 1, latch = True)
    pubVelocity = rospy.Publisher((publishedName + '/velocity'), Twist, 
            queue_size = 10)
    pubState = rospy.Publisher((publishedName + '/state_machine'), String, 
            queue_size = 1, latch = True)
    pubTargetsCollected = rospy.Publisher('targetsCollected', Int16,
            queue_size = 1, latch = True)
    pubDebug = rospy.Publisher((publishedName + '/DebugInfo'), String, 
            queue_size = 50, latch = True)
    # this is just a one time publish to get the average of all rovers to find center
    pubGPSLocation = rospy.Publisher('GPSLocation', NavSatFix,
            queue_size = 10, latch = True)
    pubName = rospy.Publisher('rover_names', String,
            queue_size = 25, latch = True)
    targetDropOffPublish  = rospy.Publisher((publishedName + "/targetDropOffImage"),Image,
            queue_size = 10, latch = True) #@DAVID WAS HERE
    targetPickupPublish = rospy.Publisher((publishedName + "/targetPickUpImage"),Image,
            queue_size = 10, latch = True) #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
            
    # timers
    timerStatus = rospy.Timer(rospy.Duration(status_publish_interval),
        pubStatusTimerCallback)

    # timer to tell the robot to actually move every 0.750*10^8  ns
    timerStateMachine = rospy.Timer(rospy.Duration(0, int(mobilityLoopTimeStep)), 
        stateStackMachine)    

    # INITIALIZATIONS
    #averageGPS = getAverageGPS()
    #pubDebug.publish(String(data = str(averageGPS.latitude) + ' ' + str(averageGPS.longitude)))
    #pubGPSLocation.publish(averageGPS)
    #pubName.publish(String(data = publishedName))
    # sleep to give the rovers time for the callback to get called
    #rospy.sleep(1.)
    #initializeRepulsionFactor()
    #initializeSubRoverLocations()
    
    #setHome()
    #PRINT
    #pubDebug.publish(String(data = 'Home is: ' + str(home.latitude) + ' : ' + str(home.longitude)))

    #pose = gpsToCarte(averageGPS)
    #pubDebug.publish(String(data = 'X: ' + str(pose.x) + ' Y: ' + str(pose.y)))
    
    # timer to update vector pointing towards home base
    timerUpdateHomeBaseForce = rospy.Timer(rospy.Duration(1),
            updateHomeForce)

    stateStack = Stack()
    goalLocation.x = goalLocation.y = 0
    stateStack.push(search)
    stateStack.push(initPose)
    #stateStack.push(swirl)
    #stateStack.push(goTo)
    #stateStack.push(initBoundaries)
    #pubDebug.publish(String(data = 'before state'))
    rospy.spin()
    return


#############################################################

################## FORCELIST FUNCTIONS ######################
'''
Addition of vector forces from global force list.
If empty returns a force of 0,0
'''
def addForces():
    
    if DEBUG_MODE:
        pubDebug.publish(String(data = "$$$DEBUG class::main => function::addForces() => message:: Fired : "))
    
    tempForce = Vector()
    for force in forceList:
        #tempForce += force # this operator was not overloaded for vectors
        tempForce.add(force)
    return tempForce

'''
Empties global force list.
'''
def clearForces():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::clearForces() => message:: Fired : "))
    global forceList
    global obstacleForce
    
    del forceList[:]

'''
Updates global force variable.
'''
def updateForce():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::updateForce() => message:: Fired : "))
    global currentForce
    currentForce = addForces()

'''
Adds force to global force list.
'''
def addForce(forceVector):
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::addForce() => message:: Fired : "))
    global forceList
    forceList.append(forceVector)
    

# NOT IMPLEMENTED
def applyRotation():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::applyRotation() => message:: Fired : "))
    return

'''
Param: takes in a gps coord (navsatfix)
Returns: a cartesian coord (pose2d)

Requires: a global variable called home that saves the home base long and lat
          geometry_msg::Pose2D
          sensor_msgs::NavSatFix
'''
def gpsToCarte(gpsCoord):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::gpsToCarte() => message:: Fired : "))
    #helper variables
    global home
    pose = Pose2D()
    #finding difference in long and lat
    pose.x = gpsCoord.longitude - home.longitude
    pose.y = gpsCoord.latitude - home.latitude

    #converting from degrees to meters
    pose.x *= gpsToCarte.longToM
    pose.y *= gpsToCarte.latToM

    #returning pose
    return pose
gpsToCarte.longToM = 97882
gpsToCarte.latToM = 110828


def movingLatAvg(x):
    global currentLatLen
    global currentLatAvg
    global movingLatList
    movingLatList.append(x)
    if currentLatLen >= avgLen:
        movingLatList.pop(0)
    else:
        currentLatLen += 1
    currentLatAvg = calcWtdAvg(movingLatList)
    return currentLatAvg 
    
def movingLongAvg(x):
    global currentLongLen
    global currentLongAvg
    global movingLongList
    movingLongList.append(x)
    if currentLongLen >= avgLen:
        movingLongList.pop(0)
    else:
        currentLongLen += 1
    currentLongAvg = calcWtdAvg(movingLongList)
    return currentLongAvg
        
 
def calcWtdAvg(myList):
    avg = 0
    for pos in range(currentLatLen):
        avg += myList[pos] * (pos + 1)
    
    denom = currentLatLen * (currentLatLen + 1) / 2
    #if denom is not 0
    if denom:
       return avg / denom
    else:
        return 0
    

#############################################################

################### FORCES ##################################
'''
    Vector pointing to the base.
    Unit vector [x,y]
    Global variable baseForce continously updated using timer.
'''
def updateHomeForce(timer):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::updateHomeForce() => message:: Fired : "))
    global baseForce
    
    baseForce = -1 * Vector(currentLocation).unit()

    #PRINT
    #pubDebug.publish(  String(data = 'Vec: ' + str(pose.x) + ' ' + str(pose.y)))
    #rospy.sleep(.5)
    #pubDebug.publish(String(data = 'UVec:' + str(baseForce[0]) + ' ' + str(baseForce[1])))
    
'''
    Initialization of global repulsionForce.
'''
def initializeRepulsionForceDic():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::initializeRepulsionForceDic() => message:: Fired : "))
    global repulsionForceDic
    for rover_name in rover_names_list:
        if publishedName != rover_name:
            repulsionForceDic[rover_name] = Vector()

'''
    Initializes repulsion factor of this rover.
    Affects the magnitude of the repulsionForce vector that gets
    updated when the repulsionForce callback gets called.
'''
def initializeRepulsionFactor():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::initializeRepulsionFactor() => message:: Fired : "))
    global repulsionFactor
    count = 0
    initialFactor = 1
    for rover in rover_names_list:
        if rover == publishedName:
            repulsionFactor = initialFactor + (count * initialFactor)
        count += 1
    repulsionFactor *= .3
    repulsionFactor += .8

# BUG LAT AND LONG ARE IN THE OTHER DIRECTION FOR SOME REASON
'''
    Creates vector pointing away from any rover within a certain distance.
    Magnitude of the vector inversely proportional to its distance.
    Updates global repulsionForce with the vector created.
'''
def repulsionForceCallback(message, roverName):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::repulsionForceCallback() => message:: Fired : "))
    otherRoverPosition = gpsToCarte(message)
    thisRoverPosition = gpsToCarte(currentGPS)
    vector = Vector(otherRoverPosition.y - thisRoverPosition.y, 
            otherRoverPosition.x - thisRoverPosition.y)
    distance = np.hypot(vector.x, vector.y)
    #PRINT
    #pubDebug.publish(String(data = 'THIS IS CALCROVERDISTANCE: ' + roverName + ' '
    #    + str(vector.x) + ' ' + str(vector.y)))
    # reverse vector direction and multiply by factor
    if distance < repulsionForceCallback.distanceConst:
        #vector *= -1 # cant' do this, no operator overloading
        vector.timesScalar(-1)
        vector.timesScalar(repulsionFactor)
        repulsionForceDic[roverName] = vector
    else:
        repulsionForceDic[roverName] = Vector()
repulsionForceCallback.distanceConst = 3

'''
    Creates a unit vector pointing within plus or minus pi/4
    its current orientaion.
'''
def wanderingForce():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::wanderingForce() => message:: Fired : "))
    randAngle = M_PI_4 * random() - M_PI_8 
    theta = currentLocation.theta + randAngle


    wanderingForce = Vector(np.cos(theta), np.sin(theta))
    return wanderingForce


def getAvoidForce():
    global obstacleForce
    goalPos = Vector(avoidGoalLocation)
    currentPos = Vector(currentLocation)
    force = goalPos - currentPos
    return force
    
def getRepulsionForce():
    repulseForce = Vector()
    for key in repulsionForceDic:
        repulseForce += repulsionForceDic[key]
        
    return repulseForce

    
################### STATESTACK FUNCTIONS#####################


def search():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::search() => message:: Fired : "))
    global goalLocation
    #adding force to force list
    force = Vector()
    #force += getBeenToForce(currentLocation.x, currentLocation.y)  
    #force += baseForce 
    force += wanderingForce()    
    force *= 0.50
 
    goalLocation.x = currentLocation.x + force.x
    goalLocation.y = currentLocation.y + force.y
    pubDebug.publish(String(data = 'I am in search! VECTOR: ' + str(currentLocation.theta)))
    pubDebug.publish(String(data = str(force)))
    #moveVector(force , 0.5, currentLocation.theta)
    stateStack.push(goTo)
    
# def spiralHome():
#     setSpeed(0.2,1)
#     return
    
def goHome():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::goHome() => message:: Fired : "))

    pubDebug.publish(String(data = 'I am going home.'))
    setVelocity(0.0,0.0)

    goalLocation.x = 0.
    goalLocation.y = 0.

    resetSwirl()
    stateStack.push(swirl)
    stateStack.push(dizzy)
    stateStack.push(goTo)
    
# def goHome():
#     if DEBUG_MODE:
#             pubDebug.publish(String(data = "$$$DEBUG class::main => function::goHome() => message:: Fired : "))
#     #clear forces
#     clearForces()
    
#     '''
#     pubDebug.publish(String(data = 'I am going home.'))
#     setVelocity(0.0,0.0)
#     averageGPS = getAverageGPS() #what's this for?
#     pose = gpsToCarte(averageGPS)
#     currentPos = Vector(pose.x,pose.y)
#     forceVector = currentPos * -1
    
#     pubDebug.publish(String(data = '***************I am at: ' + str(currentPos)))
#     pubDebug.publish(String(data = '***************Force Vector: ' + str(forceVector)))
#     '''
#     #adding force to force list
#     '''
#     addForce(baseForce)
#     for key in repulsionForceDic:
#         addForce(repulsionForceDic[key])
#     #adding all forces together
#     forceVector = addForces()
#     '''
#     goalLocation.x = 0.
#     goalLocation.y = 0.
#     stateStack.push(goTo)

def test():
    global stateStack
    global goalLocation

    pubDebug.publish(String(data = 'in test'))

    stateStack.push(goTo)
 
    test.flag = not test.flag
    if test.flag:
       goalLocation.x = goalLocation.y = 0
    else:
        goalLocation.x = goalLocation.y = 3
test.flag = True

def resetSwirl():
    #global swirl.count, swirl.mult

    swirl.count = 0
    swirl.mult = 1


def swirl():
    global stateStack, goalLocation
    #swirl reset
    if swirl.count >= 5:
        swirl.count = 0
    	swirl.mult += 1
        #continue swirling when it's trying to go home
        if not stateStack.contains(goHome):
            stateStack.popUntil(swirl)    
            return

    pubDebug.publish(String(data = 'I am swirling @'))
    #initializing vector list
    size = 0.5
    vList = [Vector(size,size), Vector(-size,size),Vector(-size,-size), Vector(size,-size), Vector(size, size)] 
    #initializing origin
    if swirl.mult == 1 and swirl.count == 0:
        swirl.origin.setVector(currentLocation)

    for pos in range(0,5):
        vList[pos] *= swirl.mult

    #setting up goal location and goto
    goalLocation.x = swirl.origin.x + vList[swirl.count].x
    goalLocation.y = swirl.origin.y + vList[swirl.count].y
    swirl.count += 1
    stateStack.push(dizzy)
    stateStack.push(goTo)

    #pubDebug.publish(String(data = 'goal: ' + str(Vector(goalLocation)) + 'current: ' + str(Vector(currentLocation)) + 'swirl: ' + str(swirl.count) + ', ' + str(swirl.mult) + 'alex'))

swirl.origin = Vector()
swirl.count = 0    
swirl.mult = 1   

def dizzy(degree = 0):
    global stateStack

    pubDebug.publish(String(data = 'I am dizzy @.@'))
    
    begin = currentLocation.theta
    setVelocity(0, -.2)
    
    radDiff = 100
    rospy.sleep(1.)

    while radDiff < (degree + 0) or radDiff > (degree + 0.1):
        radDiff = shortestAngular(begin, currentLocation.theta)

    setVelocity(0,0)
    if stateStack.peek() is dizzy:
        stateStack.pop()
    
def initPose():
    global homeOffset, stateStack, goalLocation

    off = 0.5 * Vector(np.cos(currentLocation.theta), np.sin(currentLocation.theta))
    homeOffset.x = off.x
    homeOffset.y = off.y

    goalLocation.theta = currentLocation.theta + M_PI
    goalVector = Vector(np.cos(goalLocation.theta), np.sin(goalLocation.theta))
    goalLocation.x = goalVector.x
    goalLocation.y = goalVector.y

    stateStack.pop()
    stateStack.push(goTo)
    
def goTo():
    global stateStack

    pubDebug.publish(String(data = 'I am going to.')) # not done yet
  
    force = Vector(goalLocation) - Vector(currentLocation)
    radDiff = shortestAngular(currentLocation.theta, force.radians())
    pubDebug.publish(String(data = 'goTo: ' + str(force)))

    #checks if the rover is close enough to the goal location
    if force.magnitude() < 0.05:
        pubDebug.publish(String(data = 'reached target!'))
        stateStack.pop()
        return     
    #checking if current theta is pointing to goal
    if np.fabs(radDiff) > 0.1:
        stateStack.push(rotateState)
        pubDebug.publish(String(data = 'Rotate'))
    #move forward until it hits goal state
    elif np.fabs(radDiff) < 0.3:
        stateStack.push(translateState)
        pubDebug.publish(String(data = 'Translate'))
    #check if there's a target or not
    elif targetDetected.data != -1:
        pubDebug.publish(String(data = 'Target'))
        #setting goal location to zero, if not already there
        if currentPos.magnitude() > 0.2:
            goalLocation.x = goalLocation.y = 0
            goalLocation.theta = currentPos.radians() + M_PI

'''
takes in a fromrad angle and a torad angle
returns the rad to go from fromRad to toRad
'''
def shortestAngular(fromRad, toRad):
    delta = toRad - fromRad
    
    while delta < -M_PI or delta > M_PI:
        if delta < -M_PI:
            delta += M_2_PI
        elif delta > M_PI:
            delta -= M_2_PI

    return delta

def rotateState():
    global stateStack
   
    force = Vector(goalLocation) - Vector(currentLocation) 
    radDiff = shortestAngular(currentLocation.theta, force.radians())
    
    if radDiff > 0.1:
        setVelocity(0.0, 0.2)
    elif radDiff < -0.1:
        setVelocity(0.0, -0.2)
    else:
        setVelocity(0.0, 0.0)
        stateStack.pop()
    return

def translateState():
    global stateStack
    
    force = Vector(goalLocation) - Vector(currentLocation)
    radDiff = shortestAngular(currentLocation.theta, force.radians())
    
    if np.fabs(radDiff) < 0.3:
        setVelocity(0.3, 0.0)
    else:
        setVelocity(0.0, 0.0)
        #pops to get out of translate
        stateStack.pop()

   
def avoid():
    global stateStack
    global obstacleForce

    pubDebug.publish(String(data = 'I am avoiding.')) # not done yet
 
    force = getAvoidForce()
    radDiff = shortestAngular(currentLocation.theta, force.radians())

    pubDebug.publish(String(data = 'avoid: ' + str(force)))

    #checks if the rover is close enough to the goal location
    if force.magnitude() < 0.2:
        pubDebug.publish(String(data = 'reached target!'))
        stateStack.pop()
        obstacleForce.setVector(0,0)
        return     
    #checking if current theta is pointing to goal
    if np.fabs(radDiff) > 0.1:
        stateStack.push(avoidRotate)
        pubDebug.publish(String(data = 'Rotate'))
    #move forward until it hits goal state
    elif np.fabs(radDiff) < 0.2:
        stateStack.push(avoidTranslate)
        pubDebug.publish(String(data = 'Translate'))
    #check if there's a target or not
    elif targetDetected.data != -1:
        pubDebug.publish(String(data = 'Target'))
        #setting goal location to zero, if not already there
        if currentPos.magnitude() > 0.2:
            goalLocation.x = goalLocation.y = 0
            goalLocation.theta = currentPos.radians() + M_PI

def avoidRotate():
    global stateStack
   
    force = getAvoidForce()
    radDiff = shortestAngular(currentLocation.theta, force.radians())
    
    if radDiff > 0.1:
        setVelocity(0.0, 0.2)
    elif radDiff < -0.1:
        setVelocity(0.0, -0.2)
    else:
        setVelocity(0.0, 0.0)
        stateStack.pop()
    return

def avoidTranslate():
    global stateStack
    
    force = getAvoidForce()
    radDiff = shortestAngular(currentLocation.theta, force.radians())
    
    if np.fabs(radDiff) < 0.2:
        setVelocity(0.3, 0.0)
    else:
        setVelocity(0.0, 0.0)
        #pops to get out of translate
        stateStack.pop()

def pathState():
    if DEBUG_MODE:
        pubDebug.publish(String(data = "$$$DEBUG class::main => function::pathState() => message:: Fired : "))        
    clearForces()
    for key in repulsionForceDic:
        addForce(repulsionForceDic[key])
    pathForce = Vector(pathLocation[-1])
    addForce(pathForce)
    forceVector = addForces()
    pubState.publish(String(data = "pathState State: " + str(forceVector)))
    moveVector(forceVector, 1, currentLocation.theta)
    currentPos = Vector(currentLocation)
    if currentPos.distanceFrom(pathForce) < .3:
        if len(pathLocation) > 1:
            pathLocation.pop()
        else:
            pathLocation.pop()
            stateStack.pop()

        
def initBoundaries():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::initBoundaries() => message:: Fired : "))
    for i in range(x_max):
        beenToGrid[0][i][0] = 100
        beenToGrid[0][i][1] = Vector(0,-50)
        beenToGrid[x_max-1][i][0] = 100
        beenToGrid[x_max-1][i][1] = Vector(0,50)
        beenToGrid[i][0][0] = 100
        beenToGrid[i][0][1] = Vector(50,0)
        beenToGrid[i][y_max-1][0] = 100
        beenToGrid[i][y_max-1][1] = Vector(-50,0)
    stateStack.pop()

def stateStackMachine(timer):
    #if DEBUG_MODE:
        #pubDebug.publish(String(data = "$$$DEBUG class::main => function::stateStackMachine() => message:: Fired : "))
    pubDebug.publish(String(data = 'location:' + str(Vector(currentLocation))))
    pubDebug.publish(String(data = 'theta: ' + str(currentLocation.theta)))
    if not stateStack:
        return
    if stateStack.peek() is not None and (currentMode == 2 or currentMode == 3):
        stateStack.peek()()
        #pubDebug.publish(String(data = 'CURRENT: ' + str(stateStack.peek())))
        pubDebug.publish(String(data = 'ITEMS: ' + str(stateStack.items)))
    else:
        setVelocity(0.0,0.0)
    #stateStack.peek()()# this stack should be better tested
    
#############################################################

'''
    Makes list of subscribers to different rovers' GPS location.
'''
def initializeSubRoverLocations():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::initializeSubRoverLocations() => message:: Fired : "))
    global subRoverLocationsList
    for rover in rover_names_list:
        if rover != publishedName:
            pubDebug.publish(String(data = rover + ': INITIALIZE'))
            subRoverLocationsList.append(rospy.Subscriber(rover + '/fix', NavSatFix,
                repulsionForceCallback, callback_args = rover, queue_size = 10))
            rospy.sleep(2.)


# this changes velocity of rover and published it (actually position)
def setVelocity(linear, angular):
    global msgVelocity, msgDebug     
    msgVelocity.linear.x = linear * 1.5  # [x, 0.0, 0.0]
    msgVelocity.angular.z = angular * 8.0  # [0.0, 0.0, z]
    msgVelocity.angular.x = 0.0
    msgVelocity.angular.y = 0.0
    # sends message out so odometry will get the right position
    pubVelocity.publish(msgVelocity)
    return
def setSpeed(left, right): #should be left speed and right speed from 0 to 1
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::setSpeed() => message:: Fired : "))
    global msgVelocity
    #ratio = left / right
    # ratio > 1, left arc
    # ratio < 1, right arc
    # ratio = 1, move straight
    msgVelocity.linear.x = 0.0
    msgVelocity.angular.z = 0.0 
    msgVelocity.angular.x = min(left, 1.0)
    msgVelocity.angular.y = min(right, 1.0)
    pubVelocity.publish(msgVelocity)
    
def moveVector(vec, magnitude, currentOrien=None): 
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::moveVector() => message:: Fired : "))
    """
    aheadVect needs to be generated by the currentOrient
    """
    if currentOrien == None:
        currentOrien = currentLocation.theta
    aheadVect = Vector(np.cos(currentOrien), np.sin(currentOrien))
    dist = vec.radiansDifference(aheadVect)
    #print("newVect: ", newVect)
    #magnitude = newVect.magnitude()
    if np.fabs(dist) < (3.1415926/8):
        newDist = 8*dist/3.1415926 # dist shouldn't be larger than 90, because vect is smaller than aheadVect
        if dist< 0: # assuming 0 is forward, assuming positive is turning right
            leftMotor = 1.0
            rightMotor = np.fabs(1+newDist)
        else:
            rightMotor = 1.0
            leftMotor = np.fabs(1-newDist)
    #print(leftMotor, rightMotor)
        #setSpeed(leftMotor*magnitude, rightMotor*magnitude)
	setVelocity(.3, newDist * 5)
    else:
        if dist > 0:
            setVelocity(0, 0.3)
        else:
            setVelocity(0, -0.3)

def setHome():
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::setHome() => message:: Fired : "))
    global home, homeList
    rospy.sleep(5.)
    for x in homeList:
        home.latitude = home.latitude + x.latitude
        home.longitude = home.longitude + x.longitude
        rospy.sleep(2.)
    
    home.latitude /= max(1.,len(homeList))
    home.longitude /= max(1.,len(homeList))
    
def setHomeCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::setHomeCallback() => message:: Fired : "))
    global homeList
    homeList.append(message)

def nameCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::nameCallback() => message:: Fired : "))
    global rover_names_list
    rover_names_list.append(message.data)
    rover_names_list.sort()

# updates Latitude and Longitude
# other things can be updated as well
def updateGPSCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::updateGPSCallback() => message:: Fired : "))
    global currentGPS 
    currentGPS.latitude = movingLatAvg(message.latitude)
    currentGPS.longitude = movingLongAvg(message.longitude)

# gets average lat and long
def getAverageGPS():
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::getAverageGPS() => message:: Fired : "))
    count = 0
    listLat = []
    listLong = []
    rospy.sleep(5.)
    while count < 10:
        listLat.append(currentGPS.latitude)
        listLong.append(currentGPS.longitude)
        count = count + 1
        rospy.sleep(.2)

    latitudeAvg = sum(listLat)/len(listLat)
    longitudeAvg = sum(listLong)/len(listLong)
    return NavSatFix(latitude = latitudeAvg, longitude = longitudeAvg)


def modeCallback(message):
    global currentMode
    currentMode = message.data
    setVelocity(0.0, 0.0)
    return

def pubStatusTimerCallback(timer):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::pubStatusTimerCallback() => message:: Fired : "))
    global pubStatus
    msg = String()
    msg.data = 'Synchronized Swarmies PCC'
    pubStatus.publish(msg)
    return

'''
OBSTACLE FORCE
'''
def obstacleCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::obstacleCallback() => message:: Fired : "))
    theta = 0
    global obstacleForce
    global stateStack
    global avoidGoalLocation
   
    if message.data > 0:
        if message.data == 1: #obstacle on right
            theta = currentLocation.theta + M_PI_2
        elif message.data == 2: #obstacle on left
            theta = currentLocation.theta - M_PI_2
        
        newOb = Vector(np.cos(theta), np.sin(theta))
        
        obstacleForce.setVector(newOb)
        avoidGoalLocation.x = currentLocation.x + obstacleForce.x
        avoidGoalLocation.y = currentLocation.y + obstacleForce.y
       

        if not stateStack.contains(avoid):
            #if going home just avoid
            if stateStack.contains(goHome):
                if message.data == 1:
                    theta = currentLocation.theta + M_PI_3
                else:
                    theta = currentLocation.theta - M_PI_3
                
                newOb = Vector(np.cos(theta), np.sin(theta))
        
                obstacleForce.setVector(newOb)
                avoidGoalLocation.x = currentLocation.x + obstacleForce.x
                avoidGoalLocation.y = currentLocation.y + obstacleForce.y
       
                stateStack.push(avoid)
            #it's okay if changing goal
            else:
                goalLocation.x = avoidGoalLocation.x
                goalLocation.y = avoidGoalLocation.y
        rospy.sleep(3.)
    return

# simplified version of odometryHandler from mobility.cpp/py
def odometryCallback(message):
    global currentLocation, msgDebug
    pfx = message.pose.pose # this prefix is used everywhere here!
    # postion minus homeOffset gets you the corrected currentLocation
    currentLocation.x = pfx.position.x - homeOffset.x
    currentLocation.y = pfx.position.y - homeOffset.y
    (pitch,roll,yaw) = tf.transformations.euler_from_quaternion(np.array([pfx.orientation.x, pfx.orientation.y, pfx.orientation.z, pfx.orientation.w]))
    currentLocation.theta = yaw
    return


def targetsCollectedCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::targetsCollectedCallback() => message:: Fired : "))
    global targetsCollected
    targetsCollected[message.data] = 1
    return

def joystickCallback(message):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::joystickCallback() => message:: Fired : "))
    if currentMode == 0 or currentMode == 1:
        setVelocity(message.linear.x, message.angular.z)

def shutdown(reason):
    #if DEBUG_MODE:
    #        pubDebug.publish(String(data = "$$$DEBUG class::main => function::shutdown() => message:: Fired : "))
    rospy.signal_shutdown(reason)
    return

#@purpose Keeps track of new tags and tells rover to go home.
#@param message: message object created by the camera, contains the tagID and image data
def targetCallback(message): #@DAVID WAS HERE

    pubDebug.publish(String(data = "$$$DEBUG class::main => function::targetCallback() => message:: Fired @0@0@0@0@0@0@0@0@0@0@0@0@0@0@0@0: "))

    global tagLocationList #list of queued tags with extra data
    global stateStack #machine states
    global temporaryImageData #temorarily holds the image pixel data
    global currentlyHeldTag #ID number of tag being transported home
    global targetsCollected #list of tags collected already
    global homeOffset #offset to get to home
    global goalLocation # global coordinates XY
    #global prevTagLocationXY
    global holdingTag

    if message == None:
        pubDebug.publish(String(data = "$$$DEBUG You have no messages......... "))
        return

    for i in tagLocationList:
        pubDebug.publish(String(data = str(i[0]) +" " ))

    if goHome in stateStack.items: #if you are going home...

        if homeTagID == message.tags.data[0]:# and you found the home tag
            pubDebug.publish(String(data = "$$$DEBUG Honey I'm home!!!        adding tag" +
                str(currentlyHeldTag.tags.data[0]) +
                " to the collected list...    publishing image...   Not going home anymore... "))#@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!

            #popping off states until goHome
            while stateStack.peek() is not goHome:
            	stateStack.pop()
            #pop off goHome
            stateStack.pop()
            #stateStack.push(search)
        
            currentPos = Vector(currentLocation.x, currentLocation.y)
            #found a home tag, checking to see if current location is out of a 0.5m radius
            if currentPos.magnitude() > 0.5:
                #update currentLocation to no more than 0.5m away from center
                #updating offset
                homeOffset.x += currentPos.x
                homeOffset.y += currentPos.y
                #updating currentLocation
                currentLocation.x = currentLocation.y = 0

	    if holdingTag: # if I go home with a tag #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
                #adds currentlyHeldTag to the collected list and POPS OUT THE SPECIFIC TAG in tagLocationList.
                
		pubDebug.publish(String(data = "I Dropped a tag off at home! Omg I'm so useful!"))
		targetsCollected[currentlyHeldTag.tags.data[0]] = 1 #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!

		pubTargetsCollected.publish(Int16(currentlyHeldTag.tags.data[0]))
                holdingTag = False
                targetDropOffPublish.publish(message.image) 
                temporaryImageData = message.image
                pubDebug.publish(String(data = 'home published to dropoff'))

		#If there are queued tags...
		if len(tagLocationList) != 0:
		    #then try to go to the location of the CLOSEST tag, and start spiral searching for it.
		    pubDebug.publish(String(data = "Going to search for queued tag!"))
		    #shortestTagIDPos = findShortestTagDistance(tagLocationList)
		    shortestTagIDPos = 0 
		    stackState.popUntil(search)
		    stackState.push(search)
		    
		    resetSwirl()
		    stackState.push(swirl)
		    stackState.push(dizzy)
		    stackState.push(goto)
                    goalLocation.x = tagLocationList[-1][1][0]
                    goalLocation.y = tagLocationList[-1][1][1]

                #goalLocation.x = prevTagLocationXY[0]
                #goalLocation.y = prevTagLocationXY[1]
                # stackState.push(goTo)

            rospy.sleep(1.)
            return

        else:# You found a random tag while going home
            for i in tagLocationList: #Do nothing if tag has been queued already.
                if i[0] == message.tags.data[0]:
                    pubDebug.publish(String(data = "FOUND QUEUED TAG, BUT ALREADY TRANSPORTING... IGNORING........!!!"))
                    rospy.sleep(1.)
                    return

            if targetsCollected[message.tags.data[0]] == 1: #do nothing if tag has been collected already
                pubDebug.publish(String(data = "$$$DEBUG Tag has already been collected, IGNORING......... "))
                rospy.sleep(1.)
                return

            #add random tag to the queue
            tagLocationList.append(
                (message.tags.data[0],
                (currentLocation.x,currentLocation.y),
                (currentGPS.latitude,currentGPS.longitude))) #puts tag ID number, (x,y) coordinate, and GPS coordinates into the found tags list
            rospy.sleep(1.)
            return

    else: #You are NOT going home

        if homeTagID == message.tags.data[0]:# and you found the home tag

            currentPos = Vector(currentLocation.x, currentLocation.y)
            #found a home tag, checking to see if current location is out of a 0.5m radius
            if currentPos.magnitude() > 0.5:
                #update currentLocation to no more than 0.5m away from center
                #updating offset
                homeOffset.x += currentPos.x
                homeOffset.y += currentPos.y
                #updating currentLocation
                currentLocation.x = currentLocation.y = 0

            rospy.sleep(1.)
            return

        else: # a random tag is found

            if targetsCollected[message.tags.data[0]] == 1: #do nothing if tag has been collected already
                pubDebug.publish(String(data = "$$$DEBUG Tag has already been collected, IGNORING......... "))
                return

            for i in tagLocationList: #If tag has been queued already.
                if i[0] == message.tags.data[0]:
                    pubDebug.publish(String(data = "FOUND QUEUED TAG GOING HOME!!!"))
                    currentlyHeldTag = message #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
                    holdingTag = True
                    targetPickupPublish.publish(message.image) #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
                    stateStack.push(goHome)
                    rospy.sleep(1.)
                    return

            #Brand new tag at this point
            pubDebug.publish(String(data = "FOUND NEW TAG, QUEUING AND GOING HOME!!!"))
            tagLocationList.append(
                (message.tags.data[0],
                (currentLocation.x,currentLocation.y),
                (currentGPS.latitude,currentGPS.longitude))) #puts tag ID number, (x,y) coordinate, and GPS coordinates into the found tags list
            holdingTag = True
            currentlyHeldTag = message #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
            targetPickupPublish.publish(message.image) #@DAVID WAS HERE!!!!!!!!!!!!!!!!!!!!!!!
            stateStack.push(goHome)
            rospy.sleep(1.)
            return

#roverXYPosition is a XY coordinate as a list with 2 elements, representing the rover's current location.
#tagXYPosition is a XY coordinate as a list with 2 elements, representing the location of a previously found tag.
#newTagXYPositionis a XY coordinate as a list with 2 elements, representing the new modified tag.
def tagForce(roverXYPosition, tagXYPosition, newTagXYPosition):	
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::tagForce() => message:: Fired : "))
    newTagXYPosition[0] = tagXYPosition[0] - roverXYPosition[0]#tagX - roverX
    newTagXYPosition[1] = tagXYPosition[1] - roverXYPosition[1]#tagY - roverY

#simple distance formula function
def findDistance(x1,y1,x2,y2):
    return math.sqrt( math.pow( (x2-x1), 2) + math.pow( (y2 - y1), 2) )
    
    #listOfTags must not be 0
#returns the position of the queued tagID with the shortest distance to home base
def findShortestTagDistance(listOfTags):
    if len(listOfTags) == 0:
        return -1

    shortest = float("inf")
    tagIDPosition = 0
    temp = 0.0

    for n in range(len(listOfTags)):
        temp = findDistance(listOfTags[n][1][0], listOfTags[n][1][1], 0, 0)

        if temp < shortest:
            shortest = temp
            tagIDPosition = n

    return tagIDPosition

# Been to functions
# Add x,y to beenToGrid array, and change the beenTo force for that location
def collectBeenTo(x, y):
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::collectBeenTo() => message:: Fired : "))
    global beenToGrid
    x = int(round((x * resolution) + offset))
    y = int(round((y * resolution) + offset))
    area = 3 * resolution # only apply forces within a 3x3 meter area
    beenToGrid[y][x][0] += 1 #increment position count
    vec = grabArea(x,y, area, beenToGrid)
    beenToGrid[y][x][1].add(vec)
     
# Apply beenTo force within a certain area of a point and return the force
def grabArea(x, y, area, beenToGrid):
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::grabArea() => message:: Fired : "))
    count = 0;
    forceX = 0
    forceY = 0
    for _x in range(-area,area+1):
        for _y in range(-area,area+1):
            if (_x+x) in range(x_max) and (_y+y) in range(y_max):
                if beenToGrid[_y+y][_x+x][0] != 0:
                    count += 1
                    forceX = forceX + (_x * beenToGrid[y][x][0])
                    forceY = forceY + (_y * beenToGrid[y][x][0])
    if count != 0:
        return Vector(-forceX/count, -forceY/count) # Average the forces
    else:
        return Vector(0,0)
        
# returns the force as a unit vector
def getBeenToForce(x, y):
    if DEBUG_MODE:
            pubDebug.publish(String(data = "$$$DEBUG class::main => function::getBeenToForce() => message:: Fired : "))
    return (beenToGrid[int(round((y * resolution) + offset))][int(round((x * resolution) + offset))][1]).unit()


if __name__ == '__main__':
    try:
        main(argv)
    except rospy.ROSInterruptException:
        pass
