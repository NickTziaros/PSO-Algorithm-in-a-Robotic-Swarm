#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('swarm_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
   



    while not rospy.is_shutdown():
        msg=Twist()
        msg.linear.x = 0.3 
        rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()







if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
