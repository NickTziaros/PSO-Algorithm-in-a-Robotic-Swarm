import rospy
from 	sensor_msgs.msg import PointCloud2 as pc2
from 	sensor_msgs.msg import LaserScan
from	laser_geometry 	import LaserProjection
import ros_numpy


class GetGoalPoint():
	"""docstring for GetGoalPoint"""
	def __init__(self,robotname):
		self.robotname=robotname
		self.subpc = rospy.Subscriber("/{}/LaserPointCloud".format(robotname),pc2,self.PC_callback)


	def PC_callback(self,msg):
		self.point_cloud=msg
		self.xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
	
	def PrintArray(self):
		print(self.xyz_array)	
		return self.xyz_array
