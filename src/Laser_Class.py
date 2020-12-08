#!/usr/bin/env python
import  rospy
from  math import cos,sin
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point,PointStamped,PoseStamped
import tf2_ros
import tf2_geometry_msgs



class Laser_ClosestPoint():
	
	def __init__(self,robotname):

		rate=rospy.Rate(5)
		rate.sleep()
		self.robotname=robotname	
		self.subs = rospy.Subscriber("/{}/laser_scan".format(robotname),LaserScan,self.Laser_callback)
		self.closest_point_pub = rospy.Publisher("/{}/closest_point".format(robotname),PointStamped, queue_size=10)
		self.tf_buffer=tf2_ros.Buffer(rospy.Duration(1500.0))
		self.tf_listener=tf2_ros.TransformListener(self.tf_buffer)
		self.get_tranform() 



	def get_tranform(self):
		try:
			self.transform = self.tf_buffer.lookup_transform("{}/link_chassis".format(self.robotname),"{}/sensor_laser".format(self.robotname),rospy.Time(0),rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
			rospy.logerror("Error getting Transform")
			print "ERROR"



	def Laser_callback(self,msg):
		self.laser=msg
		self.get_tranform()

	def closest_point(self):
		rate=rospy.Rate(5)
		rate.sleep()

		laser=self.laser.ranges
		shortest_laser=100000
		point=Point()
		for i in range(len(laser)):
			if laser[i]<shortest_laser:
				shortest_laser=laser[i]
				angle=self.laser.angle_min + i*self.laser.angle_increment
				x=laser[i]*cos(angle)
				point.x=x
				point.y=shortest_laser*sin(angle)
		pose=PoseStamped()
		pose.header=self.laser.header
		pose.pose.position = point

		pose_transformed= tf2_geometry_msgs.do_transform_pose(pose, self.transform)

		point_transformed=PointStamped()
		point_transformed.header=pose_transformed.header
		point_transformed.point= pose_transformed.pose.position
		self.closest_point_pub.publish(point_transformed)

		


		# returns the point closer to the robot
		return	point