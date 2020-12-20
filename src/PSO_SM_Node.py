#!/usr/bin/env python
import	rospy
import 	sys
from 	geometry_msgs.msg import Twist,Point,Pose
from 	nav_msgs.msg import Odometry
from    Laser_Class import Laser_ClosestPoint
from 	robot_Class import robot
import	smach
import  smach_ros


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
global next_point
global pub
global speed
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




class Init_Speed(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'])

 

	def execute(self,userdata):
		speed.linear.x=r.initialize_speed()
		pub.publish(speed)
		print(speed.linear.x)
		rospy.sleep(1)
		return 'finished'


class PSO(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'],output_keys=['next_point_out'])
		self.epoch=0
		self.Pbest=Point()




	def execute(self, userdata):
		obst=l.closest_point()
		obst_dist=r.euclidean_distance(obst)
		goal_dist=r.euclidean_distance(goal)
		#get Pbest
		self.Pbest=r.get_Pbest(goal)
		next_point=userdata.next_point_out=r.get_next_point(self.Pbest,Gbest,obst)
		# userdata.next_point_out=r.get_next_point(self.Pbest,Gbest,obst)
		# rospy.loginfo('X: %s Y:%s' , self.Pbest.x ,self.Pbest.y )
		self.epoch=self.epoch+1
		rospy.loginfo('Epoch:%s' , self.epoch )
		return 'finished'



class Go2Point(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished','failed','finished2'],input_keys=['next_point_in'])
		goal=Point()
		goal.x=6
		goal.y=6






	def execute(self, userdata):
			next_point=userdata.next_point_in
			# rospy.loginfo('X: %s Y:%s' , next_point.x,next_point.y )
	 		while  r.euclidean_distance(next_point)>=0.05:
	 			# rospy.loginfo('X: %s Y: %s',goal_point.x,goal_point.y )
	 			speed.angular.z=r.angular_vel(next_point)
				speed.linear.x = r.linear_vel(next_point)
				pub.publish(speed)
				# rospy.loginfo('Distance %s' ,r.euclidean_distance(goal) )
				next_point=userdata.next_point_in
				# return 'failed'
				if r.euclidean_distance(goal)<0.3:
					return 'finished2'
			return 'finished'
			
class Wait(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'])

 

	def execute(self, userdata):
		r.stop()
		rospy.sleep(10)
		return 'finished'



if __name__ == '__main__':
	args=rospy.myargv(argv=sys.argv)
	robotname= args[1]
	rospy.init_node("PSO_node".format(robotname))
	
			
	sm =smach.StateMachine(outcomes=['I_have_finished'])



	
# Create and start the introspection server for visualising state machine

	sis = smach_ros.IntrospectionServer('server_name', sm, '/{}/SM_ROOT'.format(robotname))
	sis.start()




	# Adding states
	with sm:

		smach.StateMachine.add('Init_Speed', Init_Speed(),
                                transitions={'finished': 'PSO', 'failed': 'PSO'})
		smach.StateMachine.add('PSO', PSO(),
                                transitions={'finished': 'Go2Point', 'failed': 'PSO'}, remapping={'next_point_out':'sm_next_point'})
		smach.StateMachine.add('Go2Point', Go2Point(),
                                transitions={'finished': 'PSO', 'failed': 'Go2Point', 'finished2' : 'Wait'},remapping={'next_point_in':'sm_next_point'})
		smach.StateMachine.add('Wait', Wait(),
                                transitions={'finished': 'Wait', 'failed': 'Wait'})



	# Start State_Machine
	outcome =sm.execute()
	sis.stop()
	rospy.spin()
