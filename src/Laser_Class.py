#!/usr/bin/env python
import  rospy
from  math import cos,sin
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point



class Laser_ClosestPoint():
	
	def __init__(self,robotname):
		self.subs = rospy.Subscriber("/{}/laser_scan".format(robotname),LaserScan,self.Laser_callback)
		self.pub = rospy.Publisher("/{}/closest_point".format(robotname),Point, queue_size=10)


	def Laser_callback(self,msg):
		self.laser=msg

	def closest_point(self):
		laser=self.laser.ranges
		shortest_laser=100000
		# Point=Point()
		for i in range(len(laser)):
			if laser[i]<shortest_laser:
				shortest_laser=laser[i]
				angle=self.laser.angle_min + i*self.laser.angle_increment
				x=laser[i]*cos(angle)
				# Point.x=x
				# Point.y=shortest_laser*sin(angle)
				# Point.z=0
				# self.pub.publish(Point)
		return	shortest_laser