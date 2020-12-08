#!/usr/bin/env python
import  rospy
from    geometry_msgs.msg import Twist,Point
from    nav_msgs.msg import Odometry
from    math    import sqrt,pow,atan2,pi
from    tf.transformations import euler_from_quaternion
from    Laser_Class import Laser_ClosestPoint
from    GetGoalPoint import GetGoalPoint


class robot():
    """docstring for ClassName"""
    def __init__(self,robotname):
        # self.test=Point()
        # self.test.x=-1
        # self.test.y=1
        self.robotname=robotname
        global speed 
        global pub
        global l
        speed=Twist()
        self.pc=GetGoalPoint(self.robotname)
        self.subs = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)
        self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.rate.sleep()
    def callback(self,msg):

        # getting the pose values of the robot by subscribing to the /odom topic              
        self.robot_pose_x=msg.pose.pose.position.x
        self.robot_pose_y=msg.pose.pose.position.y
        self.rot_q= msg.pose.pose.orientation
        (roll,pitch,self.yaw)= euler_from_quaternion([self.rot_q.x,self.rot_q.y,self.rot_q.z,self.rot_q.w]) 


    def closest_point(self):
        # call the Laser Class and geting the closest point
        l=Laser_ClosestPoint(self.robotname)
        point_returned=l.closest_point()
        
        closest_point=Point()
        closest_point.x=point_returned.x+self.robot_pose_x
        closest_point.y=point_returned.y+self.robot_pose_y
        return closest_point
        # return point_returned
    

    def stop(self):
        speed.linear.x = 0
        speed.angular.z = 0
        
    def euclidean_distance(self, goal_point):
        distance= sqrt(pow((self.goal_point.x - self.robot_pose_x), 2) +
                    pow((self.goal_point.y - self.robot_pose_y), 2))
        return distance

    def linear_vel(self,goal_point, constant=0.05):
        if self.euclidean_distance>4:
            return 0.25
        else:
            return constant * self.euclidean_distance(goal_point)

    def angle (self,goal_point):
        desired_angle_goal=atan2(self.goal_point.y- self.robot_pose_y,self.goal_point.x- self.robot_pose_x)
        return desired_angle_goal

    def angular_vel(self, goal_point, constant=1):
        angle_diff=self.angle(goal_point) - self.yaw
        if abs(angle_diff)>0.05:
            if angle_diff!=2*pi:
                speed.angular.z=angular_vel=-constant * (angle_diff)
            else:
                speed.angular.z=angular_vel=constant * (angle_diff)
            rospy.loginfo('angular_vel %s, %s',angular_vel,angle_diff)  
            angle_diff=self.angle(goal_point) - self.yaw 
            self.pub.publish(speed)
        # return angular_vel
    
    def go2goal(self):
        
        while not rospy.is_shutdown() :
            # goal_point= the point closer to the robot that the Class Laser_class returns
            self.goal_point=self.closest_point()
            # self.goal_point=self.test


# -------------------------------------------------------------------------
            # My go2goal
            while  self.euclidean_distance(self.goal_point)>=0.5:
                self.pc.PrintArray()
                self.angular_vel(self.goal_point)
                speed.linear.x = self.linear_vel(self.goal_point)
                rospy.loginfo('X: %s Y: %s',self.goal_point.x,self.goal_point.y )
                rospy.loginfo('X: %s Y: %s',self.robot_pose_x,self.robot_pose_y )
                rospy.loginfo('distance: %s',self.euclidean_distance(self.goal_point))
            # Publishing our vel_msg
                self.pub.publish(speed)
                self.goal_point=self.closest_point()

            return True  
            rospy.spin()


# -------------------------------------------------------------------------

    def avoid_obstacle(self):
    
        speed.linear.x=-0.1
        speed.angular.z=-0.1
        self.pub.publish(speed)
