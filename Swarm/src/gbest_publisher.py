#!/usr/bin/env python
import 	rospy
import 	sys
from    nav_msgs.msg import Odometry
from 	geometry_msgs.msg import Twist,Point,Pose
from    tf.transformations import euler_from_quaternion
from 	robot_Class import get_goal
from    math    import sqrt,pow
import 	message_filters


def euclidean_distance( goal_point_x,goal_point_y,robot_pose_x, robot_pose_y):
	distance= sqrt(pow((goal_point_x - robot_pose_x), 2) +
	pow((goal_point_y - robot_pose_y), 2))
	return distance
def callback(msg1,msg2,msg3,msg4,msg5):
	distance_buffer=[]
	distance_buffer.insert(0,euclidean_distance(goal.x,goal.y,msg1.pose.pose.position.x,msg1.pose.pose.position.y))
	distance_buffer.insert(1,euclidean_distance(goal.x,goal.y,msg2.pose.pose.position.x,msg2.pose.pose.position.y))
	distance_buffer.insert(2,euclidean_distance(goal.x,goal.y,msg3.pose.pose.position.x,msg3.pose.pose.position.y))
	distance_buffer.insert(3,euclidean_distance(goal.x,goal.y,msg4.pose.pose.position.x,msg4.pose.pose.position.y))
	distance_buffer.insert(4,euclidean_distance(goal.x,goal.y,msg5.pose.pose.position.x,msg5.pose.pose.position.y))
	# distance_buffer.insert(5,euclidean_distance(goal.x,goal.y,msg6.pose.pose.position.x,msg6.pose.pose.position.y))
	# distance_buffer.insert(6,euclidean_distance(goal.x,goal.y,msg7.pose.pose.position.x,msg7.pose.pose.position.y))
	# distance_buffer.insert(7,euclidean_distance(goal.x,goal.y,msg8.pose.pose.position.x,msg8.pose.pose.position.y))
	# distance_buffer.insert(8,euclidean_distance(goal.x,goal.y,msg9.pose.pose.position.x,msg9.pose.pose.position.y))
	Gbest=10000
	robot=0
	for i in range(0,5):
		if distance_buffer[i]<Gbest:

			Gbest=distance_buffer[i]
			robot=i
	
	# rospy.loginfo('distance: %s %s' ,Gbest,robot)
	if robot==0: 
		Gbest_pose.position.x=msg1.pose.pose.position.x
		Gbest_pose.position.y=msg1.pose.pose.position.y
		Gbest_pose.orientation=msg1.pose.pose.orientation
	if robot==1:
		Gbest_pose.position.x=msg2.pose.pose.position.x
		Gbest_pose.position.y=msg2.pose.pose.position.y
		Gbest_pose.orientation=msg2.pose.pose.orientation
	if robot==2:
		Gbest_pose.position.x=msg3.pose.pose.position.x
		Gbest_pose.position.y=msg3.pose.pose.position.y
		Gbest_pose.orientation=msg3.pose.pose.orientation
	if robot==3:
		Gbest_pose.position.x=msg4.pose.pose.position.x
		Gbest_pose.position.y=msg4.pose.pose.position.y
		Gbest_pose.orientation=msg4.pose.pose.orientation
	if robot==4:
		Gbest_pose.position.x=msg5.pose.pose.position.x
		Gbest_pose.position.y=msg5.pose.pose.position.y
		Gbest_pose.orientation=msg5.pose.pose.orientation
	# if robot==5:
	# 	Gbest_pose.position.x=msg6.pose.pose.position.x
	# 	Gbest_pose.position.y=msg6.pose.pose.position.y
	# 	Gbest_pose.orientation=msg6.pose.pose.orientation
	# if robot==6:
	# 	Gbest_pose.position.x=msg7.pose.pose.position.x
	# 	Gbest_pose.position.y=msg7.pose.pose.position.y
	# 	Gbest_pose.orientation=msg7.pose.pose.orientation
	# if robot==7:
	# 	Gbest_pose.position.x=msg8.pose.pose.position.x
	# 	Gbest_pose.position.y=msg8.pose.pose.position.y
	# 	Gbest_pose.orientation=msg8.pose.pose.orientation
	# if robot==8:
	# 	Gbest_pose.position.x=msg9.pose.pose.position.x
	# 	Gbest_pose.position.y=msg9.pose.pose.position.y
	# 	Gbest_pose.orientation=msg9.pose.pose.orientation
	pub.publish(Gbest_pose)


rospy.init_node('Gbest_publisher')	
rate = rospy.Rate(10)
rate.sleep()
pub = rospy.Publisher("get_Gbest",Pose, queue_size=10)
Gbest_pose=Pose()
global goal
goal=Point()
goal.x,goal.y=get_goal()
print(goal)
sub_1=message_filters.Subscriber('/robot_1/odom',Odometry)
sub_2=message_filters.Subscriber('/robot_2/odom',Odometry)
sub_3=message_filters.Subscriber('/robot_3/odom',Odometry)
sub_4=message_filters.Subscriber('/robot_4/odom',Odometry)
sub_5=message_filters.Subscriber('/robot_5/odom',Odometry)
# sub_6=message_filters.Subscriber('/robot_6/odom',Odometry)
# sub_7=message_filters.Subscriber('/robot_7/odom',Odometry)
# sub_8=message_filters.Subscriber('/robot_8/odom',Odometry)
# sub_9=message_filters.Subscriber('/robot_9/odom',Odometry)

ts = message_filters.ApproximateTimeSynchronizer([sub_5,sub_4,sub_3,sub_2,sub_1], 10, 0.1, allow_headerless=True)
ts.registerCallback(callback)

# while not rospy.is_shutdown() :

rospy.spin()
