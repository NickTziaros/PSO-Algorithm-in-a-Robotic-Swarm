import  rospy
from    geometry_msgs.msg import Twist,Point
from    nav_msgs.msg import Odometry




class robot():
    """docstring for ClassName"""
    def __init__(self,robotname):
        # rate=rospy.rate(10)
        global speed 
        global pub
        speed=Twist()
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)

        

    def go(self):
        speed.linear.x=0.05
        speed.angular.z=0.0
        self.pub.publish(speed)

    def turn_left(self):
        speed.linear.x=0.05
        speed.angular.z=-0.8
        self.pub.publish(speed)
    def turn_right(self):
        speed.linear.x=0.05
        speed.angular.z=0.8
        self.pub.publish(speed)
    def stop(self):
        speed.linear.x=0.0
        speed.angular.z=0.0
        self.pub.publish(speed)
    def avoid_obstacle(self):
    
        speed.linear.x=0.1
        speed.angular.z=-0.1
        self.pub.publish(speed)