# #!/usr/bin/env python
# import	rospy
# import 	sys
# import 	math
# from 	geometry_msgs.msg import Twist,Point
# from 	nav_msgs.msg import Odometry
# import	smach
# import  smach_ros
# import	time
# from 	robot_Class import robot
# from    Laser_Class import Laser_ClosestPoint


	
# args=rospy.myargv(argv=sys.argv)
# robotname= args[1]

# # States of state machine
# #----------------------------------------------------------------------------------------------------------

# #Define Approach State
# class Approach(smach.State):
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['finished','failed'])
# 		# na kanw to r meros ths classhs
# 		global r
# 		global l
# 		global speed
# 		r=robot(robotname)
# 		speed=Twist()
# 		# Initializes the Laser_ClosestPoint class
# 		l=Laser_ClosestPoint(robotname)
# 		self.pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)


# 	def execute(self, userdata):

# 		goal_point=l.closest_point()
#  		while  r.euclidean_distance(goal_point)>=1:
#  			rospy.loginfo('X: %s Y: %s',goal_point.x,goal_point.y )
#  			speed.angular.z=r.angular_vel(goal_point)
# 			speed.linear.x = r.linear_vel(goal_point)
# 			pub.publish(speed)
# 			rospy.loginfo('X: %s' ,r.euclidean_distance(goal_point) )
# 			goal_point=l.closest_point()
# 		rospy.loginfo('X: %s Y: %s',goal_point.x,goal_point.y )	
# 		return 'finished'



# #Define Wait State
# class Wait(smach.State):
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['finished', 'failed'])

 

# 	def execute(self, userdata):
# 		r.stop()
# 		rospy.sleep(3)
# 		return 'finished'
		
# #Define Repel State
# class Repel(smach.State):
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['finished', 'failed'])

 

# 	def execute(self, userdata):
# 		return 'finished'





# # Initializations of SM && publishers
# # -------------------------------------------------------------------------------------------------------------
# if __name__ == '__main__':		


# # Initialize node
# 	rospy.init_node('State_machine_node')
# 	sm =smach.StateMachine(outcomes=['I_have_finished'])
# 	pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)


	
# # Create and start the introspection server for visualising state machine

# 	sis = smach_ros.IntrospectionServer('server_name', sm, '/{}/SM_ROOT'.format(robotname))
# 	sis.start()




# 	# Adding states
# 	with sm:
# 		smach.StateMachine.add('Approach', Approach(),
#                                 transitions={'finished': 'Wait', 'failed': 'Approach'})
# 		smach.StateMachine.add('Repel', Repel(),
#                                 transitions={'finished': 'Approach', 'failed': 'Repel'})
# 		smach.StateMachine.add('Wait', Wait(),
#                                 transitions={'finished': 'Repel', 'failed': 'Wait'})
# 		# smach.StateMachine.add('Attract',CBState(Attract_state_callback),{'finished':'Repulse', 'failed':'Attract'})
# 		# smach.StateMachine.add('Repulse',CBState(Repulse_state_callback),{'finished':'Attract', 'failed':'Repulse'})



# 	# Start State_Machine
# 	outcome =sm.execute()
# 	sis.stop()
# 	rospy.spin()









