#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point,Pose
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi
from    tf.transformations import euler_from_quaternion
from    Laser_Class import Laser_ClosestPoint
from    GetGoalPoint import GetGoalPoint
from    random import *
import  numpy

class robot():
    """docstring for ClassName"""
    def __init__(self,robotname):
        self.test=Point()
        self.test.x=10
        self.test.y=10
        self.robotname=robotname
        global speed 
        global pub

        self.Pbest_point=Point()
        self.next_point=Point()
        speed=Twist()
        self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.rate.sleep()
        global w
        w=1.05
        global c1
        c1=2
        global c2
        c2=2
        global c3
        c3=0
        self.Pbest_point.x=self.robot_pose_x
        self.Pbest_point.y=self.robot_pose_y
    # callback from the subscriber

    def callback(self,msg):

        # getting the pose values of the robot by subscribing to the /odom topic              
        self.robot_pose_x=msg.pose.pose.position.x
        self.robot_pose_y=msg.pose.pose.position.y
        self.rot_q= msg.pose.pose.orientation
        (roll,pitch,self.yaw)= euler_from_quaternion([self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w]) 

    # def get_pose(self):
    #     RobotPosition=Pose()
    #     RobotPosition.point.x=self.robot_pose_x
    #     RobotPosition.point.y=self.robot_pose_y
    #     RobotPosition.orientation=self.rot_q
    #     return RobotPosition

    def closest_point(self):
        # call the Laser Class and geting the closest point
        
        point_returned=l.closest_point()
        
        closest_point=Point()
        closest_point.x=point_returned.point.x
        closest_point.y=point_returned.point.y
        return closest_point

    def get_Pbest(self,goal_point):
        
        if self.euclidean_distance(goal_point)<self.euclidean_distance(self.Pbest_point): 
            self.Pbest_point.x=self.robot_pose_x
            self.Pbest_point.y=self.robot_pose_y
        return self.Pbest_point

    def get_next_point(self,Gbest,Pbest,obst):
        self.next_point.x=w*(self.robot_pose_x)+c1*numpy.random.uniform(0,1)*(Pbest.x-self.robot_pose_x)+c2*numpy.random.uniform(0,1)*(Gbest.x-self.robot_pose_x)-c3*numpy.random.uniform(0,1)*(obst.x-self.robot_pose_x)
        self.next_point.y=w*(self.robot_pose_y)+c1*numpy.random.uniform(0,1)*(Pbest.y-self.robot_pose_y)+c2*numpy.random.uniform(0,1)*(Gbest.y-self.robot_pose_y)-c3*numpy.random.uniform(0,1)*(obst.y-self.robot_pose_y)
        rospy.loginfo('%s' , self.robotname )
        rospy.loginfo(' Pbest X: %s Y:%s' , Gbest.x,Gbest.y )
        rospy.loginfo('Gbest X: %s Y:%s' , Pbest.x,Pbest.y )
        rospy.loginfo('next_point X: %s Y:%s' , self.next_point.x,self.next_point.y )
        return self.next_point
# initializes Speed randomly in range of (0.05,1)     
# -------------------------------------------------------------------------  
    def initialize_speed(self):
        speed.linear.x=numpy.random.uniform(0.05,1)
        self.pub.publish(speed)
        return speed.linear.x

# -------------------------------------------------------------------------  
    def stop(self):
        speed.linear.x = 0
        speed.angular.z = 0
        self.pub.publish(speed)


# Gets the distance to the  point given as argument
# -------------------------------------------------------------------------  
        
    def euclidean_distance(self, goal_point):
        distance= sqrt(pow((goal_point.x - self.robot_pose_x), 2) +
                    pow((goal_point.y - self.robot_pose_y), 2))

        return distance

# -------------------------------------------------------------------------  

    def linear_vel(self,goal_point, constant=0.07):
        if self.euclidean_distance>4:
            return 0.3
        else:
            return constant * self.euclidean_distance(goal_point)


# -------------------------------------------------------------------------  

    def angle (self,goal_point):
        desired_angle_goal=atan2(goal_point.y- self.robot_pose_y,goal_point.x- self.robot_pose_x)
        return desired_angle_goal

# -------------------------------------------------------------------------  

    def angular_vel(self, goal_point, constant=4):
        angle_diff=self.angle(goal_point) - self.yaw
        if abs(angle_diff)>0.1:
            if angle_diff!=0:
                speed.angular.z=angular_vel=-constant * (angle_diff)
            else:
                speed.angular.z=angular_vel=constant * (angle_diff)
            # rospy.loginfo('angular_vel %s, %s',angular_vel,angle_diff)  
            # angle_diff=self.angle(goal_point) - self.yaw 

        return speed.angular.z



# -------------------------------------------------------------------------    
    # def go2goal(self):
        
    #     while not rospy.is_shutdown() :
    #         # goal_point= the point closer to the robot that the Class Laser_class returns
    #         self.goal_point=self.closest_point()
    #         # self.goal_point=self.test




    #         while  self.euclidean_distance(self.goal_point)>=1:
    #             # self.pc.PrintArray()
    #             self.angular_vel(self.goal_point)
    #             speed.linear.x = self.linear_vel(self.goal_point)
    #             rospy.loginfo('X: %s Y: %s',self.goal_point.x,self.goal_point.y )
    #             # rospy.loginfo('X: %s Y: %s',self.robot_pose_x,self.robot_pose_y )
    #             rospy.loginfo('distance: %s',self.euclidean_distance(self.goal_point))
    #         # Publishing our vel_msg
    #             self.pub.publish(speed)
    #             self.goal_point=self.closest_point()

    #         return True  
    #         # rospy.spin()


# -------------------------------------------------------------------------

    def avoid_obstacle(self):
    
        speed.linear.x=-0.1
        speed.angular.z=-0.1
        self.pub.publish(speed)
