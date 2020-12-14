#!/usr/bin/env python
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point
from 	nav_msgs.msg import Odometry
from    Laser_Class import Laser_ClosestPoint
from 	robot_Class import robot




args=rospy.myargv(argv=sys.argv)
robotname= args[1]

global l
global r
l=Laser_ClosestPoint(robotname)
r=robot(robotname)
self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
# define_goal
goal=Point()
goal.x=10
goal.y=10
Pbest=10000
# Define PSO Parameters w,c1,r1,c2,r2







while not rospy.is_shutdown() :
	obst=l.closest_point()
	obst_dist=r.euclidean_distance(obst)
	goal_dist=r.euclidean_distance(goal)
	#get Pbest
	if goal_dist+obst_dist<Pbest:
		Pbest=goal_dist


	#get Gbest




	# Update Velocity
	# V=w*V+c1*r1*(Pbest-X)+c2*r2(Gbest-X)
















if __name__ == '__main__':
	rospy.init_node("PSO_node")
	rospy.spin()