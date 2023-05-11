from nav_msgs.msg import Odometry
import rospy

class poser:

    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback)

    def callback(self, msg):
        self.pose = msg.pose.pose
