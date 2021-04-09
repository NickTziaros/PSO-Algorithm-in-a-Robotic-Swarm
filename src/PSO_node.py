#!/usr/bin/env python3
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from    Laser_Class import Laser_ClosestPoint
from 	robot_Class import robot
from    math    import sqrt,pow,atan2,pi



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
# Gbest=Point()
# # define_goal
speed=Twist()
goal=Point()
goal.x=6
goal.y=-6
# Pbest=10000
# next_point=Point()
# # Define PSO Parameters w,c1,r1,c2,r2
sub=rospy.Subscriber('/get_Gbest',Pose,callback)
pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)
epoch=0


while not rospy.is_shutdown() :
		obst=l.closest_point()
		obst_dist=r.euclidean_distance(obst)
		goal_dist=r.euclidean_distance(goal)
		#get Pbest
		angle=r.angle(obst)
		angle_goal=r.angle(goal)

		goal_angular_speed=r.angular_vel_deg(goal)

		goal_linear_speed =r.linear_vel(goal)
		# r.linear_vel(goal)
		obst_angular_speed=r.get_apf_vel(goal)
		# rospy.loginfo('obst %s' ,obst_angular_speed )
		# rospy.loginfo('goal %s' , goal_angular_speed )
		speed.linear.x=goal_linear_speed
		speed.angular.z=goal_angular_speed+obst_angular_speed
		# speed.linear.x=0
		# speed.angular.z=0
		# print(speed.angular.z)



# ----------------------
		# print(round(r.yaw * (180 / pi), 4))
		# angle_degrees = round(angle * (180 / pi), 4);
		# angle_gdegrees = round(angle_goal * (180 / pi), 4);
		# angle_diff=angle_gdegrees-round(r.yaw * (180 / pi), 4)
		# if abs(angle_diff)>1:
		# 	if angle_diff>90:
		# 		speed.angular.z= -0.05*(angle_gdegrees-round(r.yaw * (180 / pi), 4))
		# 	else:
		# 		speed.angular.z= -0.05*(angle_gdegrees-round(r.yaw * (180 / pi), 4))
		# 	print(angle_diff)
		# 	pub.publish(speed)
		# print(angle_diff)
		# rospy.loginfo('%s' , angle_degrees )
		# rospy.loginfo('g     %s' , angle_gdegrees )
#-------------------------	
		# rospy.loginfo('X: %s Y: %s',goal_point.x,goal_point.y )
		# speed.angular.z=r.angular_vel(next_point)
		# speed.linear.x = r.linear_vel(next_point)
		pub.publish(speed)
		# rospy.loginfo('Distance %s' ,r.euclidean_distance(goal) )


if __name__ == '__main__':
	rospy.spin()
