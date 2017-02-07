import rospy
from sys import argv
from nav_msgs.msg import Odometry

def callback(msg):
    prefix = msg.pose.pose
    print("x: %f y: %f" %(prefix.position.x, prefix.position.y))
    print("x: %f, y: %f, z: %f, w: %f" %(prefix.orientation.x, prefix.orientation.y, prefix.orientation.z, prefix.orientation.w))

def main(argv):
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(argv[1]+'/odom/ekf', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        if len(argv) > 1:
            main(argv)
    except rospy.ROSInterruptException:
        pass
