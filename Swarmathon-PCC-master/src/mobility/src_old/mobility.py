import rospy
from sys import argv

#Socket is used to import the host name we need so that when a subscriber or
#publisher is called it can add the relative path to host name to get the
#socket. For example if the host name is alpha the mobility publisher would
#write to the alpha/mobility topic
import socket

#Libraries in Place of ROS libraries
import random #in place of random_number.h
import math
import tf

#Ros Messages
from std_msgs.msg import UInt8, String, Int16
from sensor_msgs.msg import Joy, Range
from geometry_msgs.msg import Pose2D, Point, Twist, Quaternion
from nav_msgs.msg import Odometry

# This is the message type we will be modifying, and posting
# to different topics to give the rovers different velocities.
mobility = Twist() #Global Velocity Message
prev_state_machine = String('WAITING')

#Numeric variables
currentLocation = Pose2D()
goalLocation = Pose2D()
currentMode = 0
mobilityLoopTimeStep = 0.125
status_publish_interval = 5
targetDetected = Int16()
targetsCollected = [0] * 256 #Array of Booleans

# State Machine States
# These are the constant globals that define what state of the rover
# Depending on what state the rover is in it will do something different
# You can modify what the rover does in the different states in the
# mobilityStateMachine function
STATE_MACHINE_TRANSFORM = 0
STATE_MACHINE_ROTATE = 1
STATE_MACHINE_TRANSLATE = 2
stateMachineState = STATE_MACHINE_TRANSFORM

#Publishers
# These are defined in the main, but declared here so that other functions can call publish
# Not sure if this is the best way to do it.
global mobilityPublish
global stateMachinePublish
global statusPublisher
global targetCollectedPublisher

def main(argv):
    global goalLocation, targetDetected
    global mobilityPublish, stateMachinePublish, statusPublisher, targetCollectedPublisher

    hostname = socket.gethostname()

    goalLocation.theta = random.random() * 2 * math.pi

    targetDetected.data = -1

    goalLocation.x = 0.5 * math.cos(goalLocation.theta)
    goalLocation.y = 0.5 * math.sin(goalLocation.theta)

    if len(argv) >= 2:
        publishedName = argv[1]
        print ('Welcome to the world of tomorrow', publishedName, '! Mobility module started.')
    else:
        publishedName = hostname
        print ('No Name Selected. Default is:', publishedName)

    rospy.init_node((publishedName + '_MOBILITY'), argv, disable_signals=True)


    # Publishers
    # The first parameter defines where the message will be published
    # For example if the publishedName string is alpha then mobilityPublish
    # will publish to the alpha/mobility topic
    # The second parameter is the type of message that it will publish when
    # the respective publisher gets called.
    # For example mobilityPublish.publish will write a msg of type Twist to a given topic
    mobilityPublish = rospy.Publisher((publishedName + '/mobility'), Twist, queue_size=10)
    stateMachinePublish = rospy.Publisher((publishedName + '/state_machine'), String, queue_size=1, latch=True)
    statusPublisher = rospy.Publisher((publishedName + '/status'), String, queue_size=1,latch=True)
    targetCollectedPublisher = rospy.Publisher((publishedName + '/status'), Int16, queue_size=1, latch=True)


    # Subscribers
    # First and second parameters are the same as the Publisher parameters
    # The main difference is that a subscriber is LISTENING to a topic for a message to get posted
    # Once it reads one it sends the message to the third parameter which is a function that
    #   can handle it.
    # Individual call_back functions are still not defined, must define and then replace the name of the
    # function with the callback_function parameter
    joySubscriber = rospy.Subscriber((publishedName + '/joystick'), Joy, \
            joyCmdHandler, queue_size=10)
    modeSubscriber = rospy.Subscriber((publishedName + '/mode'), UInt8, \
            modeHandler, queue_size=1)
    targetSubscriber = rospy.Subscriber((publishedName + '/targets'), Int16, \
            targetHandler, queue_size=10)
    obstacleSubscriber = rospy.Subscriber((publishedName + '/obstacle'), UInt8, \
            obstacleHandler, queue_size=10)
    odometrySubscriber = rospy.Subscriber((publishedName + '/odom/ekf'), Odometry, \
            odometryHandler, queue_size=10)
    targetsCollectedSubscriber = rospy.Subscriber('targetsCollected', Int16, \
            targetsCollectedHandler, queue_size=10)

    publish_status_timer = rospy.timer.Timer(rospy.Duration(status_publish_interval), publishStatusTimerEventHandler)
    stateMachineTimer = rospy.timer.Timer(rospy.Duration(int(math.ceil(mobilityLoopTimeStep))), mobilityStateMachine)

    rospy.spin()


#Mobility Logic Functions

# This publishes the velocity of the rover
def setVelocity(linearVel, angularVel):
    global mobilityPublish
    global mobility
    mobility.linear.x = linearVel * 1.5 # these are scales from c++ code
    mobility.angular.z = angularVel * 8
    mobilityPublish(mobility)

# This function attempts to get the shortest angular distance between two angles
def shortest_angular_distance(initialAngle, targetAngle):
    return math.atan2(math.sin(targetAngle - initialAngle), math.cos(targetAngle - initialAngle))

###############################################################
# ROS CALLBACK HANDLERS
###############################################################
def targetsCollectedHandler(message):
    global targetsCollected
    targetsCollected[message.data] = 1

def modeHandler(message):
    global currentMode
    currentMode = message.data
    setVelocity(0.0, 0.0)

def joyCmdHandler(message):
    global currentMode, mobility, mobilityPublish
    if currentMode == 0 or currentMode == 1:
        mobility.angular.z = message.angular.z * 8
        mobility.linear.x = message.angular.x * 1.5
        mobilityPublish.publish(mobility)

# Prints status to the /status topic
# Not sure if the timer is used at all or if the timer parameter is needed here
# But it's how it was in the cpp code so I'll leave it here as well
def publishStatusTimerEventHandler(timer):
    global statusPublisher
    msg = String()
    msg.data = 'online'
    statusPublisher.publish(msg)

# Function that checks the target and then goes back to the center if target hasn't been collected.
def targetHandler(message):
    global targetDetected, targetsCollected, targetCollectedPublish, currentLocation
    global stateMachineState, STATE_MACHINE_TRANSFORM
    if targetDetected.data == -1:
        targetDetected = message
        if not targetsCollected[targetDetected.data]:
            goalLocation.theta = math.pi + math.atan2(currentLocation.y, currentLocation.x)

            # set center as goal position
            goalLocation.x = 0.0
            goalLocation.y = 0.0

            # publish detected target
            targetCollectedPublish.publish(targetDetected)

            # switch to transform state to trigger return to center
            stateMachineState = STATE_MACHINE_TRANSFORM

# Function that gets a theta related to roll, pitch, yaw transformations
# But since a rover can only rotate about the z axis we only use the yaw transformation
# Since I don't know about the math behind this I'm not sure if this function works
def odometryHandler(message):
    global currentLocation
    currentLocation.x = message.pose.pose.position.x
    currentLocation.y = message.pose.pose.position.y

    q = Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y, \
            message.pose.pose.orientation.z, message.pose.pose.orentation.w)
    euler = tf.transformations.euler_from_quaternion(q)
    yaw = euler[2]
    currentLocation.theta = yaw

# Function that sets the goal location depending on where the obstacle is in front of you.
# Then sets the stateMachineState to TRANSFORM so that the stateMachine could take care of it.
def obstacleHandler(message):
    global goalLocation, currentLocation
    if message.data > 0:
        # obstacle on right side
        if message.data == 1:
            # select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.2
        # obstacle in front or on left side
        elif message.data == 2:
            goalLocation.theta = currentLocation.theta - 0.2

        # select new position 50 cm from current location
        goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta))
        goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta))

        # switch to transform state to trigger collision avoidance
        stateMachineState = STATE_MACHINE_TRANSFORM

##########################################################################################
# END CALLBACK FUNCTIONS
##########################################################################################

def mobilityStateMachine(timer):
    global STATE_MACHINE_TRANSFORM, STATE_MACHINE_TRANSLATE, STATE_MACHINE_ROTATE
    global stateMachineState, goalLocation, currentLocation, currentMode, targetDetected
    global prev_state_machine
    stateMachineMsg = String()

    if currentMode == 2 or currentMode == 3:
        # CASE STATE_MACHINE_TRANSFORM
        if stateMachineState == STATE_MACHINE_TRANSFORM:
            # If angle between current and goal is significant
            # Using atan2(sin(x-y), cos(x-y)) to get the shortest angular distance
            if math.fabs(shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1:
                stateMachineState = STATE_MACHINE_ROTATE
            # If goal not yet reached
            elif math.fabs(shortest_angular_distance(currentLocation.theta, math.atan2(goalLocatio.y - \
                    currentLocation.y, goalLocation.x - currentLocation.x))) < (math.pi / 2):
                stateMachineState = STATE_MACHINE_TRANSLATE
            # If returning with a target
            elif not targetDetected.data == -1:
                # If goal has not yet been reached
                if math.hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.2:
                    # Set angle to center as goal heading
                    goalLocation.theta = math.pi + math.atan2(currentLocation.y, currentLocation.x)

                    # Set center as goal position
                    goalLocation.x = 0.0
                    goalLocation.y = 0.0
                # Otherwise, reset target and select new random uniform heading
                else:
                    targetDetected.data = -1
                    goalLocation.theta = random.random() * 2 * math.pi
            # Otherwise, assign a new goal
            else:
                # Select new heading from Gaussian distribution around current heading
                goalLocation.theta = random.gauss(currentLocation.theta, 0.25)

                # Select new postion 50cm from current location
                goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta))
                goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta))

        # CASE STATE_MACHINE_ROTATE
        # Purposefully not elif
        if stateMachineState == STATE_MACHINE_ROTATE:
            stateMachineMsg.data = 'ROTATING'
            if shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1:
                setVelocity(0.0, -0.2) # rotate left
            elif shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1:
                setVelocity(0.0, -0.2) # rotate right
            else:
                setVelocity(0.0, 0.0) # stop
                stateMachineState = STATE_MACHINE_TRANSLATE

        # CASE STATE_MACHINE_TRANSLATE
        # Calculate angle between currentLocation.x/y and goalLocation.x/y
        # Drive forward
        # Stay in this state until angle is at least Pi/2
        elif stateMachineState == STATE_MACHINE_TRANSLATE:
            stateMachineMsg.data = 'TRANSLATING'

            if(math.fabs(shortest_angular_distance(currentLocation.theta, math.atan2(goalLocation.y - \
                    currentLocation.y, goalLocation.x - currentLocation.x))) < (math.pi / 2)):
                setVelocity(0.3, 0.0)
            else:
                setVelocity(0.0, 0.0)
                stateMachineState = STATE_MACHINE_TRANSFORM
    else: # mode is NOT auto
        stateMachineMsg.data = 'WAITING'

    if not stateMachineMsg.data == prev_state_machine:
        stateMachinePublish.publish(stateMachineMsg)
        prev_state_machine = stateMachineMsg.data



# Initiates a clean shutdown. The reason is just a string defining the reason for shutdown
def shutdown(reason):
    rospy.signal_shutdown(reason)

if __name__ == '__main__':
    try:
        main(argv)
    except rospy.ROSInterruptException:
        pass
