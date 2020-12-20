#!/usr/bin/env python
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from    Laser_Class import Laser_ClosestPoint
from 	robot_Class import robot



def get_Pbest():
	pass
def callback(msg):

	Gbest.x=msg.position.x
	Gbest.y=msg.position.y

def euclidean_distance( goal_point_x,goal_point_y,robot_pose_x, robot_pose_y):
	distance= sqrt(pow((goal_point_x - robot_pose_x), 2) +
	pow((goal_point_y - robot_pose_y), 2))
	return distance


args=rospy.myargv(argv=sys.argv)
robotname= args[1]
rospy.init_node("PSO_node")
global l
global r
l=Laser_ClosestPoint(robotname)
r=robot(robotname)
Gbest=Point()
# define_goal
speed=Twist()
goal=Point()
goal.x=6
goal.y=6
Pbest=10000
next_point=Point()
# Define PSO Parameters w,c1,r1,c2,r2
sub=rospy.Subscriber('/get_Gbest',Pose,callback)
pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
epoch=0


while not rospy.is_shutdown() :
	while  r.euclidean_distance(next_point)>=0.4:
		obst=l.closest_point()
		obst_dist=r.euclidean_distance(obst)
		goal_dist=r.euclidean_distance(goal)
		#get Pbest
		Pbest=r.get_Pbest(goal)
		next_point=r.get_next_point(Pbest,Gbest,obst)

		rospy.loginfo('X: %s Y:%s' , next_point.x,next_point.y )
		epoch=epoch+1
		rospy.loginfo('Epoch:%s' , epoch )

		# rospy.loginfo('X: %s Y: %s',goal_point.x,goal_point.y )
		speed.angular.z=r.angular_vel(next_point)
		speed.linear.x = r.linear_vel(next_point)
		pub.publish(speed)
		rospy.loginfo('Distance %s' ,r.euclidean_distance(goal) )

	r.stop()

if __name__ == '__main__':
	rospy.spin()