#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point
from    nav_msgs.msg import Odometry
import  math
from    tf.transformations import euler_from_quaternion
from    Laser_Class import Laser_ClosestPoint



class robot():
    """docstring for ClassName"""
    def __init__(self,robotname):
        
        self.robotname=robotname
        global speed 
        global pub
        global l
        speed=Twist()
        self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)

    def go2goal(self,x_goal,y_goal):
        
        while not rospy.is_shutdown() :
            K_linear=0.1
            distance=abs(math.sqrt(((x_goal-self.robot_pose_x) ** 2) + ((y_goal-self.robot_pose_y) ** 2)))
            speed.linear.x=K_linear*distance
            K_angular=1.5
            desired_angle_goal=math.atan2(y_goal- self.robot_pose_y,x_goal- self.robot_pose_x)
            rospy.loginfo('robot_name is: %s ', desired_angle_goal-self.yaw)
            # Den prolavainei na mpei sto callback kai varaei error!! gia 
            # auto exei rate 10Hz
            rate=rospy.Rate(10)
            rate.sleep()




# -------------------------------------------------------------------------
            # My go2goal
            if  distance>1 :
                rospy.loginfo('robot_name is: %s ', distance)
                if abs(desired_angle_goal- self.yaw)>=0.2 :
                    if desired_angle_goal- self.yaw!=0 :
                    
                        self.turn_left()
                    
                    else:
                        self.turn_right()
                else:          
            
                    self.go()
            else:
                return True
            # if distance<0.2:
            #     break
                            
        return True    

# -------------------------------------------------------------------------


    def callback(self,msg):

        # getting the pose values of the robot by subscribing to the /odom topic              
        self.robot_pose_x=msg.pose.pose.position.x
        self.robot_pose_y=msg.pose.pose.position.y
        self.rot_q= msg.pose.pose.orientation
        (roll,pitch,self.yaw)= euler_from_quaternion([self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w]) 


    def closest_point(self):
        # call the Laser Class and geting the closest point
        l=Laser_ClosestPoint(self.robotname)
        closest_point=l.closest_point()
        return closest_point

    def go(self):
        speed.linear.x=0.2
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